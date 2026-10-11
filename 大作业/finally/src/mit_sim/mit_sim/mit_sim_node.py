#!/usr/bin/env python3
"""MuJoCo 仿真控制节点。

订阅 12 个电机的 MIT 参数（/mit_cmd）。命令里的 tau 已经由控制器按
    tau = kp*(q_des - q) + kd*(dq_des - dq)
算好（不含重力补偿），本节点只负责叠加自己负责的偏置：
    tau_cmd = cmd.tau + qfrc_bias
（qfrc_bias 由参数 gravity_compensation 控制是否叠加，且命令里 kp 全 0 时不叠加；
 力矩限幅由模型 ctrlrange = ±33.5 自动完成）

推进后把关节状态发布回 /mit_state：
    q   = data.qpos[jnt_qposadr]       关节位置
    dq  = data.qvel[jnt_dofadr]        关节速度
    ddq = data.qacc[jnt_dofadr]        关节加速度（求解器结果）
    tau = data.actuator_force          实际施加的关节力矩（已限幅）

物理推进放在独立线程里按绝对时间以 500 Hz 实时运行，mj_step 与 viewer.sync()
都在该线程内完成；ROS 回调只负责更新命令缓存。
"""

import os
import sys
import threading
import time
from typing import List, Optional

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu

from mit_interfaces.msg import MitCmd, MitState

try:  # 无图形环境下导入失败不应该影响节点启动
    import mujoco.viewer

    VIEWER_AVAILABLE = True
except Exception:  # pragma: no cover - 取决于运行环境
    VIEWER_AVAILABLE = False

NUM_MOTORS = 12

DEFAULT_INITIAL_Q = [
    0.0, 1.2,  -2.5,
    0.0, -1.2, 2.5,
    0.0, -1.2, 2.5,
    0.0, 1.2, -2.5,
]


def make_qos(best_effort: bool, depth: int) -> QoSProfile:
    """构造 QoS；best-effort 与 reliable 必须与对端保持一致，否则收不到数据。"""
    return QoSProfile(
        depth=depth,
        reliability=(
            ReliabilityPolicy.BEST_EFFORT if best_effort else ReliabilityPolicy.RELIABLE
        ),
        durability=DurabilityPolicy.VOLATILE,
    )


def viewer_usable() -> bool:
    """判断当前环境能否打开可视化窗口。

    GLFW 初始化失败时 MuJoCo 会直接终止进程，无法用异常捕获，所以先做一次预检；
    这样在 ssh / 无显示环境下会退化为无界面运行，而不是把节点打挂。
    """
    if not VIEWER_AVAILABLE:
        return False
    if sys.platform.startswith("linux") and not (
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    ):
        return False
    try:
        import glfw

        return bool(glfw.init())
    except Exception:
        return False


def get_sensor_data(model,data,sensor_name):
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
    if sensor_id < 0:
        raise ValueError(f"找不到传感器 {sensor_name}")
    adr = model.sensor_adr[sensor_id] #该传感器数据在data.sensordata中的起始下标
    dim = model.sensor_dim[sensor_id] #表示该传感器数据占多少个元素，即维度长度
    return data.sensordata[adr:adr+dim].copy() #左闭右开，包含起始下标adr,不包含结束下标adr+dim

def read_imu(model,data,prefix='trunk'):
    gyro = get_sensor_data(model,data,f'{prefix}_gyro')
    accel = get_sensor_data(model,data,f'{prefix}_accel')
    quat = get_sensor_data(model,data,f'{prefix}_quat')
    return gyro,accel,quat

class MitSimNode(Node):
    def __init__(self):
        super().__init__("mit_sim_node")

        # ---------- 参数 ----------
        model_path = self.declare_parameter("model_path", "").value
        initial_q = list(self.declare_parameter("initial_q", DEFAULT_INITIAL_Q).value)
        self.gravity_compensation = bool(
            self.declare_parameter("gravity_compensation", True).value
        )
        publish_rate = float(self.declare_parameter("publish_rate", 500.0).value)
        self.real_time = bool(self.declare_parameter("real_time", True).value)
        self.cmd_timeout = float(self.declare_parameter("cmd_timeout", 0.2).value)
        use_viewer = bool(self.declare_parameter("viewer", True).value)
        self.keyboard_control = bool(self.declare_parameter("keyboard_control", True).value)
        cmd_topic = self.declare_parameter("cmd_topic", "/mit_cmd").value
        state_topic = self.declare_parameter("state_topic", "/mit_state").value
        key_topic = self.declare_parameter("key_topic", "/key_input").value
        cmd_best_effort = bool(self.declare_parameter("cmd_best_effort", True).value)
        state_best_effort = bool(self.declare_parameter("state_best_effort", True).value)

        if not model_path:
            model_path = os.path.join(os.getcwd(), "black_description.xml")
        if not os.path.isfile(model_path):
            raise RuntimeError(
                "找不到模型文件：%s（可用参数 model_path 指定，或在工作区根目录下运行）" % model_path
            )
        if len(initial_q) != NUM_MOTORS:
            raise RuntimeError(
                "参数 initial_q 需要 %d 个元素，实际为 %d" % (NUM_MOTORS, len(initial_q))
            )
        if publish_rate <= 0.0:
            raise RuntimeError("参数 publish_rate 必须为正")

        # ---------- 模型与下标映射（与 black_description.xml 的 actuator 顺序一致） ----------
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.q_adrs = np.zeros(NUM_MOTORS, dtype=int)#对应每个关节在data.qpos的下标
        self.dq_adrs = np.zeros(NUM_MOTORS, dtype=int)#对应每个关节在data.qvel的下标
        self.act_ids = np.zeros(NUM_MOTORS, dtype=int)
        for i in range(NUM_MOTORS):
            jnt_id = self.model.actuator_trnid[i, 0]
            self.q_adrs[i] = self.model.jnt_qposadr[jnt_id]
            self.dq_adrs[i] = self.model.jnt_dofadr[jnt_id]
            self.act_ids[i] = i

        self.data.qpos[self.q_adrs] = np.asarray(initial_q, dtype=float)
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

        self.dt = float(self.model.opt.timestep)
        self.publish_every = max(1, int(round((1.0 / publish_rate) / self.dt)))

        # ---------- 线程间共享的输入 ----------
        self._lock = threading.Lock()
        self._latest_cmd: Optional[MitCmd] = None
        self._cmd_stamp = 0.0
        self._pending_key: Optional[int] = None
        self._step_count = 0
        self._running = True

        # ---------- 接口 ----------
        self.cmd_sub = self.create_subscription(
            MitCmd, cmd_topic, self._on_cmd, make_qos(cmd_best_effort, 1)
        )
        self.state_pub = self.create_publisher(
            MitState, state_topic, make_qos(state_best_effort, 1)
        )
        # 窗口按键原样转发给控制器，由控制器决定发哪一组 MIT 参数
        self.key_pub = self.create_publisher(Int32, key_topic, 1)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 1)

        # ---------- 可视化（与物理推进同线程使用） ----------
        self.viewer = None
        if use_viewer:
            if viewer_usable():
                try:
                    self.viewer = mujoco.viewer.launch_passive(
                        self.model, self.data, key_callback=self._on_key
                    )
                except Exception as ex:  # pragma: no cover - 取决于运行环境
                    self.get_logger().warn("打开可视化窗口失败，改为无界面运行：%s" % ex)
            else:
                self.get_logger().warn(
                    "当前环境无法打开 MuJoCo 可视化窗口（无显示或 GLFW 不可用），改为无界面运行"
                )

        self.get_logger().info(
            "仿真节点已启动：模型 %s，步长 %.4f s，发布 %s（每 %d 步，%s），订阅 %s（%s），"
            "按键转发 %s，重力补偿 %s（kp 全 0 时自动不叠加），实时推进 %s，可视化 %s"
            % (
                os.path.basename(model_path),
                self.dt,
                state_topic,
                self.publish_every,
                "best-effort" if state_best_effort else "reliable",
                cmd_topic,
                "best-effort" if cmd_best_effort else "reliable",
                key_topic,
                "开" if self.gravity_compensation else "关",
                "开" if self.real_time else "关（尽快推进）",
                "开" if self.viewer is not None else "关",
            )
        )

        self._thread = threading.Thread(target=self._step_loop, name="mit_sim_step", daemon=True)
        self._thread.start()

    # ---------- ROS 回调（执行器线程） ----------

    def _publish_imu(self, gyro, accel, quat) -> None:
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_site"

        msg.orientation.w = quat[0]
        msg.orientation.x = quat[1]
        msg.orientation.y = quat[2]
        msg.orientation.z = quat[3]

        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]

        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]

        self.imu_pub.publish(msg)

    def _on_cmd(self, msg: MitCmd) -> None:
        with self._lock:
            self._latest_cmd = msg
            self._cmd_stamp = time.monotonic()

    def _on_key(self, keycode: int) -> None:
        """可视化窗口按键：把按键数字原样转发，含义由控制器解释（8=站姿，9=松弛）。"""
        if not self.keyboard_control:
            return
        key = chr(keycode)
        if not key.isdigit():
            return
        with self._lock:
            self._pending_key = int(key)

    # ---------- 物理推进线程 ----------

    def _step_loop(self) -> None:
        next_deadline = time.monotonic() + self.dt
        while self._running and rclpy.ok():
            if self.real_time:
                left = next_deadline - time.monotonic()
                if left > 0:
                    time.sleep(left)
            self._step_once()
            next_deadline += self.dt
            if self.real_time:
                now = time.monotonic()
                if now - next_deadline > self.dt:  # 落后过多时重新对齐，不补跑积压
                    next_deadline = now + self.dt

    def _step_once(self) -> None:
        with self._lock:
            cmd = self._latest_cmd
            command_age = time.monotonic() - self._cmd_stamp if cmd is not None else float("inf")
            pending_key = self._pending_key
            self._pending_key = None

        if pending_key is not None:
            self.key_pub.publish(Int32(data=pending_key))

        if cmd is None or command_age > self.cmd_timeout:
            # 没有有效命令时按零力矩运行，避免控制器掉线后仍保持高位保持力
            self.data.ctrl[:] = 0.0
        else:
            # tau 已由控制器按公式算好（不含重力补偿），这里只叠加偏置
            tau = np.asarray(cmd.tau, dtype=float).copy()
            # kp 全为 0 视为松弛（电机放开）：不叠加重力补偿，
            # 否则机器狗会被 qfrc_bias 托住，无法瘫下
            limp = not np.any(np.asarray(cmd.kp))
            if self.gravity_compensation and not limp:
                tau = tau + self.data.qfrc_bias[self.dq_adrs]
            # 模型 ctrllimited=true 会把 ctrl 钳位到 ±33.5
            self.data.ctrl[self.act_ids] = tau

        mujoco.mj_step(self.model, self.data)

        self._step_count += 1
        if self._step_count % self.publish_every == 0:
            self._publish_state()

        if self.viewer is not None and self._step_count %8 ==0:
            if not self.viewer.is_running():
                self.get_logger().info("可视化窗口已关闭，仿真节点退出")
                self._running = False
                rclpy.try_shutdown()
                return
            self.viewer.sync()

    def _publish_state(self) -> None:
        msg = MitState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base"
        msg.q = self.data.qpos[self.q_adrs].tolist()
        msg.dq = self.data.qvel[self.dq_adrs].tolist()
        msg.ddq = self.data.qacc[self.dq_adrs].tolist()
        msg.tau = self.data.actuator_force[self.act_ids].tolist()
        msg.cur = np.zeros(NUM_MOTORS, dtype=float).tolist()
        self.state_pub.publish(msg)
        self._publish_imu(*read_imu(self.model,self.data,prefix='trunk'))

    # ---------- 退出 ----------

    def stop(self) -> None:
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None


def main(args: Optional[List[str]] = None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = MitSimNode()
    except Exception as ex:
        print("仿真节点启动失败：%s" % ex)
        rclpy.shutdown()
        return 1

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
