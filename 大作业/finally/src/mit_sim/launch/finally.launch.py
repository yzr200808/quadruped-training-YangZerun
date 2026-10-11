"""一键启动整套系统：MuJoCo 仿真节点 + rl_sar 控制器节点（+ 手柄驱动）

怎么用（工作空间根目录 = /home/yzr/work/大作业/finally）：
    colcon build --packages-select mit_interfaces rl_sim mit_sim
    source install/setup.bash
    ros2 launch mit_sim finally.launch.py

可选参数（举几个常用的）：
    ros2 launch mit_sim finally.launch.py gravity_compensation:=false
        # 关掉仿真端的重力/科氏补偿，只留纯 PD（更贴近策略训练时的条件，建议对比试一次）
    ros2 launch mit_sim finally.launch.py run_rl_sim:=false
        # 不在这里启动控制器，改成另开一个终端 ros2 run rl_sar rl_sim（这样键盘输入一定接得上）
    ros2 launch mit_sim finally.launch.py viewer:=false
        # 不打开 MuJoCo 可视化窗口（无显示环境用）

操作按键（键盘要在跑 rl_sim 的那个终端里按；手柄直接按键即可）：
    0 / A         起身：从当前姿态插值到 default_dof_pos
    1 / RB+十字上 进入 RL，神经网络开始控制
    9 / B         趴下（插值回初始姿态）
    P / LB+X      放松（电机卸力）
    W/S、A/D、Q/E 或左右摇杆 → 前后 / 左右 / 转向速度指令；空格把指令清零
"""

# ---------------------------------------------------------------------------
# 【1】导入区：这里是包含各种库
#      launch 框架、节点描述、参数类型、手柄设备检测、路径工具
# ---------------------------------------------------------------------------
import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# ---------------------------------------------------------------------------
# 【2】默认值：MuJoCo 模型文件位置 + 是否插着手柄
#      （模型里的 meshdir 是相对路径，所以 meshes/ 必须和这个 xml 同级）
# ---------------------------------------------------------------------------
DEFAULT_MODEL_PATH = "/home/yzr/work/大作业/finally/black_description.xml"

JOY_DEVICES = sorted(glob.glob("/dev/input/js*"))
DEFAULT_JOY = "true" if JOY_DEVICES else "false"
JOY_NOTE = (
    "检测到手柄设备 %s，启动 joy_node" % "、".join(JOY_DEVICES)
    if JOY_DEVICES
    else "未检测到 /dev/input/js*，不启动 joy_node；"
    "可用 ros2 topic pub --once /joy sensor_msgs/msg/Joy \"{buttons: [0, 1], axes: [0,0,0,0,0,0,0,0]}\" 模拟手柄"
)


def generate_launch_description():
    # -----------------------------------------------------------------------
    # 【3】读取 mit_sim 的默认配置（话题名、发布频率、QoS 都在那个 yaml 里）
    #      旧的 mit_controller_cpp 不再启动 —— 它就是被下面的 rl_sim 替换掉的控制器
    # -----------------------------------------------------------------------
    sim_config = os.path.join(
        get_package_share_directory("mit_sim"), "config", "mit_sim.yaml"
    )

    return LaunchDescription(
        [
            # ---------------------------------------------------------------
            # 【4】声明可在命令行覆盖的参数（不改 yaml 也能临时调）
            # ---------------------------------------------------------------
            DeclareLaunchArgument(
                "robot_name",
                default_value="black",
                description="机器人名，决定用 policy/<名字>/ 下的配置和状态机",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=DEFAULT_MODEL_PATH,
                description="MuJoCo 模型 xml 路径",
            ),
            DeclareLaunchArgument(
                "viewer", default_value="true", description="是否打开 MuJoCo 可视化窗口"
            ),
            DeclareLaunchArgument(
                "real_time", default_value="true", description="是否按 500 Hz 实时推进物理"
            ),
            DeclareLaunchArgument(
                "gravity_compensation",
                default_value="true",
                description="仿真端是否叠加 qfrc_bias 重力补偿（RL 建议试 false 做对比）",
            ),
            DeclareLaunchArgument(
                "run_rl_sim",
                default_value="false",
                description="是否由本 launch 启动控制器；false 时请自己 ros2 run rl_sar rl_sim",
            ),
            DeclareLaunchArgument(
                "joy",
                default_value=DEFAULT_JOY,
                description="是否启动 joy 包的手柄驱动（默认按是否插着手柄自动决定）",
            ),
            LogInfo(msg=JOY_NOTE),

            # ---------------------------------------------------------------
            # 【5】仿真节点：MuJoCo 物理推进，发布 /mit_state、/imu/data
            #      参数用命令行覆盖 yaml 里的值（模型路径、是否可视化、重力补偿等）
            # ---------------------------------------------------------------
            Node(
                package="mit_sim",
                executable="mit_sim_node",
                name="mit_sim_node",
                output="screen",
                parameters=[
                    sim_config,
                    {
                        "model_path": ParameterValue(
                            LaunchConfiguration("model_path"), value_type=str
                        ),
                        "viewer": ParameterValue(
                            LaunchConfiguration("viewer"), value_type=bool
                        ),
                        "real_time": ParameterValue(
                            LaunchConfiguration("real_time"), value_type=bool
                        ),
                        "gravity_compensation": ParameterValue(
                            LaunchConfiguration("gravity_compensation"), value_type=bool
                        ),
                    },
                ],
            ),

            # ---------------------------------------------------------------
            # 【6】控制器节点：rl_sar 的 rl_sim
            #      订阅 /mit_state、/imu/data、/joy，跑神经网络，发布 /mit_cmd
            #      emulate_tty 让终端里的按键界面正常显示（键盘输入若无效，用
            #      run_rl_sim:=false + 单独 ros2 run rl_sar rl_sim 的方式）
            # ---------------------------------------------------------------
            Node(
                package="rl_sar",
                executable="rl_sim",
                name="rl_sim_node",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(LaunchConfiguration("run_rl_sim")),
                parameters=[
                    {
                        "robot_name": ParameterValue(
                            LaunchConfiguration("robot_name"), value_type=str
                        )
                    }
                ],
            ),

            # ---------------------------------------------------------------
            # 【7】手柄驱动：读 /dev/input/js* 并发布 /joy，rl_sim 订阅它来做
            #      速度指令和状态切换（所以不需要再单独跑 xbox_send 节点了）
            # ---------------------------------------------------------------
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("joy")),
            ),
        ]
    )
