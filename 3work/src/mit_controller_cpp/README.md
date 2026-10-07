# mit_controller_cpp

四足 MIT 参数控制器节点：按窗口按键决定发哪一组 12 电机 MIT 参数
`{kp, kd, q, dq, tau}`（500 Hz 发布），物理仿真由另一个节点负责并回传状态。
节点之间传输的始终只有这五个数组。

按键语义（窗口按键由仿真节点经 `/key_input` 转发过来，控制器按内置表解释）：

| 按键 | 行为 | 发出的参数 |
| --- | --- | --- |
| `8` | 站姿保持 | `kp/kd` 取参数，`q` 从当前实测关节角插值到站姿，`dq=0`，`tau` = 按公式算出的 PD 力矩（不含重力补偿） |
| `9` | 松弛（电机放开） | `kp=0`，`kd=limp_kd`(0.01)，`q=0`，`dq=0`，`tau` = 只剩阻尼力矩 `kd*(0-dq)` |

`tau` 字段是**控制器按公式算好的 PD 力矩**（不含重力补偿）：
`tau = kp*(q_des - q) + kd*(dq_des - dq)`，其中 `q/dq` 取最新一帧 `/mit_state` 反馈；
重力补偿由仿真节点在收到命令后自行叠加。

开机即松弛；旧按键 9 对应的"趴下固定姿态"已删除，趴下姿态只作为仿真节点的初始关节角。

## 数据流

```
mit_controller_node ──/mit_cmd (MitCmd)──▶ 仿真/驱动节点
                    ◀──/mit_state (MitState)──
       ▲
       ├──/key_input  (std_msgs/Int32, 窗口按键数字，8=站姿 9=松弛)
       └──/target_pose (std_msgs/Int32, 2=站姿，脚本/外部触发)
```

与原 `dog.py` 的对应关系：

| dog.py | 本节点 |
| --- | --- |
| `kp` / `kd` 数组 | 参数 `kp` / `kd`，原样放进消息 |
| `pose_2`（站姿）+ 键盘 8 | 参数 `stand_pose`；按 8 或 `/target_pose=2` 触发 |
| 键盘 9 趴下姿态 | 已删除，改为松弛模式（`kp` 全 0，仿真侧不叠重力补偿） |
| `s = 10t³-15t⁴+6t⁵` 插值 | 同一个五次多项式，时长参数 `blend_time` |
| `dq_des` 恒为 0 | 消息 `dq` 字段恒为 0 |
| `data.qfrc_bias` 重力补偿 | 由仿真节点内部叠加（见 `mit_sim` 包） |
| `tau = kp*(q_des-q) + kd*(dq_des-dq) + tau_gravity` | PD 部分由本节点算好放进 `tau`；仿真节点再加上 `qfrc_bias` 后写入 `data.ctrl` |
| 限幅 `±33.5` | 模型 `ctrlrange` 自动完成（`ctrllimited="true"`） |

## 话题

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/mit_cmd` | `mit_interfaces/MitCmd` | 发布 | 12 组 MIT 参数，500 Hz |
| `/mit_state` | `mit_interfaces/MitState` | 订阅 | 回传仿真状态 `q` / `dq` / `ddq` / `tau` |
| `/key_input` | `std_msgs/Int32` | 订阅 | 窗口按键数字：8=站姿，9=松弛；其他值告警忽略 |
| `/target_pose` | `std_msgs/Int32` | 订阅 | 只接受 2（站姿），等效按 8；其他值告警忽略 |

数组下标顺序固定为 `black_description.xml` 中 actuator 的顺序：
`0-2 = FL hip/thigh/calf`，`3-5 = FR`，`6-8 = RR`，`9-11 = RL`。

> **QoS 注意**：`/mit_cmd` 与 `/mit_state` 默认都用 best-effort + keep-last-1，
> 两端必须保持一致（参数 `cmd_best_effort` / `state_best_effort`），否则 DDS 不匹配、
> 静默收不到任何数据。

## 参数

见 `config/mit_controller.yaml`：`kp`、`kd`、`stand_pose`（站姿目标）、`limp_kd`(0.01)、
`blend_time`(3 s)、`publish_rate`(500 Hz)、`feedback_timeout`(0.1 s)、
`cmd_topic`、`state_topic`、`key_topic`、`target_pose_topic`、
`cmd_best_effort`、`state_best_effort`、`use_sim_time`。

`stand_pose` 里前腿大腿给了 ±0.3 rad 的前倾（后腿本来就是 ±0.6）：前腿大腿与地面的夹角
从 76° 降到 68°、小腿从 38° 变成 55°，不再是近乎垂直的立柱，按 9 松弛时重力立刻在
膝关节产生力矩，趴平耗时从 2.4 s 降到 0.6 s 左右；站姿高度只升高约 2 cm。

## 运行

```bash
cd /home/yzr/work/3work
colcon build --symlink-install
source install/setup.bash
ros2 launch mit_controller_cpp mit_controller.launch.py
```

只发布命令、不接仿真时，可以用命令行模拟反馈：

```bash
ros2 topic pub -r 500 /mit_state mit_interfaces/msg/MitState \
  "{q: [0.0,1.2,-2.5, 0.0,-1.2,2.5, 0.0,-1.2,2.5, 0.0,1.2,-2.5],
    dq: [0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0],
    ddq: [0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0],
    tau: [0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0]}" \
  --qos-reliability best_effort
ros2 topic pub --once /target_pose std_msgs/msg/Int32 "{data: 2}"
ros2 topic pub --once /key_input std_msgs/msg/Int32 "{data: 8}"   # 8=站姿，9=松弛
ros2 topic echo /mit_cmd --qos-reliability best_effort
```

## 与仿真节点联调

MuJoCo 仿真节点在独立的功能包 `mit_sim` 里，可一键同时启动两者：

```bash
ros2 launch mit_sim sim_with_controller.launch.py            # 带可视化
ros2 launch mit_sim sim_with_controller.launch.py viewer:=false
```

站姿保持这一路与直接运行 `dog.py` 等价（重力补偿由仿真节点叠加）。

## 安全行为

- 收到首帧反馈前：发全 0（视作松弛）；此时按 8 会先挂起，等首个状态到达后
  再以实测关节角为起点插值，避免命令从 0 起跳。
- 超过 `feedback_timeout`（默认 100 ms）没有反馈：发全 0，仿真侧同样按松弛处理。
