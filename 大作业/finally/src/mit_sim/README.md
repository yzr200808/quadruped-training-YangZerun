# mit_sim

MuJoCo 仿真控制节点：接收 12 个电机的 MIT 参数，在 MuJoCo 中推进物理，并把关节状态
发回控制器。它是原先 `mit_controller_cpp/scripts/sim_bridge.py` 的正式节点版本。

## 数据流

```
mit_controller_node ──/mit_cmd (MitCmd)──▶ mit_sim_node
                    ◀──/mit_state (MitState)──
       ▲
       └──/key_input (std_msgs/Int32, 窗口按键数字原样转发)
```

命令里的 `tau` 已经由控制器按 `tau = kp*(q_des - q) + kd*(dq_des - dq)` 算好
（不含重力补偿），本节点只负责叠加自己负责的偏置（限幅交给模型 `ctrlrange = ±33.5`）：

```
tau_cmd = cmd.tau + qfrc_bias
```

其中 `qfrc_bias` 是重力/科氏偏置项，由参数 `gravity_compensation` 控制是否叠加；
**收到命令里 `kp` 全为 0 时视为松弛（按键 9），本节点不会叠加 `qfrc_bias`**，
否则机器狗会被重力补偿托住、无法瘫下。控制器的 `tau_ff` 默认为 0，所以重力补偿完全由本节点负责。

## 话题

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/mit_cmd` | `mit_interfaces/MitCmd` | 订阅 | 12 组 MIT 参数（`kp/kd/q/dq/tau`） |
| `/mit_state` | `mit_interfaces/MitState` | 发布 | `q` / `dq` / `ddq` / `tau` |
| `/key_input` | `std_msgs/Int32` | 发布 | 窗口按键数字（8/9…），语义由控制器解释 |
| `/imu/data` | `sensor_msgs/Imu` | 发布 | MuJoCo 传感器测得的姿态/角速度/加速度 |

`/mit_state` 的物理含义：`q = data.qpos`、`dq = data.qvel`、`ddq = data.qacc`、
`tau = data.actuator_force`（实际施加且已按 `ctrlrange` 限幅的关节力矩）。
数组下标顺序为 `0-2 FL hip/thigh/calf`，`3-5 FR`，`6-8 RR`，`9-11 RL`。

> **QoS 注意**：`/mit_cmd` 与 `/mit_state` 默认都用 best-effort + keep-last-1，
> 两端（控制器与仿真节点）必须保持一致，否则 DDS 不匹配、静默收不到数据。
> 需要改用 reliable 时，同时修改两个节点的 `cmd_best_effort` / `state_best_effort`。

## 参数

见 `config/mit_sim.yaml`：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `model_path` | `/home/yzr/work/3work/black_description.xml` | 模型文件，`meshdir` 相对该文件解析 |
| `initial_q` | `dog.py` 的 `pose_1`（趴下） | 初始关节角 |
| `gravity_compensation` | `true` | 是否叠加 `qfrc_bias`（`kp` 全 0 时自动不叠加） |
| `publish_rate` | `500.0` | 状态发布频率 |
| `real_time` | `true` | 按实时推进；`false` 为尽快推进 |
| `cmd_timeout` | `0.2` | 超时未收到命令则按零力矩运行 |
| `viewer` | `true` | 可视化窗口，无图形环境自动退化 |
| `keyboard_control` | `true` | 窗口按键原样转发到 `/key_input` |
| `key_topic` | `/key_input` | 按键转发话题 |

## 运行

```bash
cd /home/yzr/work/3work
colcon build --symlink-install
source install/setup.bash

# 控制器 + 仿真一起启动（带可视化窗口）
ros2 launch mit_sim sim_with_controller.launch.py

# 无界面运行
ros2 launch mit_sim sim_with_controller.launch.py viewer:=false

# 单独启动仿真节点（未给 model_path 时，默认取当前目录下的 black_description.xml）
ros2 run mit_sim mit_sim_node --ros-args -p viewer:=false \
  -p model_path:=$PWD/black_description.xml
```

启动后在窗口里按 `8` 站起、按 `9` 松弛；无窗口时直接发话题：

```bash
ros2 topic pub --once /key_input std_msgs/msg/Int32 "{data: 8}"   # 8=站姿，9=松弛
ros2 topic echo /mit_state --qos-reliability best_effort
ros2 topic echo /imu/data
```

## 手柄控制

`xbox_send_node` 订阅 `/joy`（joy 包的手柄驱动发布），按键边缘检测后往 `/key_input` 发指令：
**A 键 → 9 松弛，B 键 → 8 站姿**。

launch 会按是否插着手柄自动决定要不要起 `joy_node`：

- 插着手柄（存在 `/dev/input/js*`）：自动启动 `joy_node`，直接用手柄即可；
- 没插手柄：跳过 `joy_node`（避免报错刷屏），手柄节点照常运行，可以用下面两种方式模拟。

```bash
# 手工发 Joy 消息模拟按键（按下 + 松开；边缘检测需要一松一按才会再次触发）
# buttons = [A, B, X, Y]，B 键（索引 1）= 站姿，A 键（索引 0）= 松弛
ros2 topic pub --once /joy sensor_msgs/msg/Joy "{buttons: [0, 1], axes: [0,0,0,0,0,0,0,0]}"
ros2 topic pub --once /joy sensor_msgs/msg/Joy "{buttons: [0, 0], axes: [0,0,0,0,0,0,0,0]}"

# 强制开关手柄驱动
ros2 launch mit_sim sim_with_controller.launch.py joy:=true    # 有手柄但没自动认出来
ros2 launch mit_sim sim_with_controller.launch.py joy:=false   # 没有手柄
```

## 依赖

`mujoco`、`numpy` 通过 pip 安装（非 ROS 包）：

```bash
pip install mujoco numpy
```
