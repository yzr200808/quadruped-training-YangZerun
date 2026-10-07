"""同时启动 MIT 控制器节点与 MuJoCo 仿真节点。"""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 模型位于工作区根目录（含 meshes/），不复制进功能包
DEFAULT_MODEL_PATH = "/home/yzr/work/3work/black_description.xml"

# 插着手柄（/dev/input/js*）才默认启动 joy_node；没插就用命令行模拟
JOY_DEVICES = sorted(glob.glob("/dev/input/js*"))
DEFAULT_JOY = "true" if JOY_DEVICES else "false"
JOY_NOTE = (
    "检测到手柄设备 %s，启动 joy_node" % "、".join(JOY_DEVICES)
    if JOY_DEVICES
    else "未检测到 /dev/input/js*，不启动 joy_node；"
    "可用 ros2 topic pub --once /joy sensor_msgs/msg/Joy \"{buttons: [0, 1], axes: [0,0,0,0,0,0,0,0]}\" 模拟手柄"
)


def generate_launch_description():
    controller_config = os.path.join(
        get_package_share_directory("mit_controller_cpp"), "config", "mit_controller.yaml"
    )
    sim_config = os.path.join(
        get_package_share_directory("mit_sim"), "config", "mit_sim.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path", default_value=DEFAULT_MODEL_PATH, description="MuJoCo 模型 xml 路径"
            ),
            DeclareLaunchArgument(
                "viewer", default_value="true", description="是否打开 MuJoCo 可视化窗口"
            ),
            DeclareLaunchArgument(
                "real_time", default_value="true", description="是否按 500 Hz 实时推进物理"
            ),
            DeclareLaunchArgument(
                "joy",
                default_value=DEFAULT_JOY,
                description="是否启动 joy 包的手柄驱动节点（默认按是否插着手柄自动决定）",
            ),
            LogInfo(msg=JOY_NOTE),
            Node(
                package="mit_controller_cpp",
                executable="mit_controller_node",
                name="mit_controller_node",
                output="screen",
                parameters=[controller_config],
            ),
            Node(
                package="mit_sim",
                executable="mit_sim_node",
                name="mit_sim_node",
                output="screen",
                parameters=[
                    sim_config,
                    {
                        "model_path": ParameterValue(LaunchConfiguration("model_path"), value_type=str),
                        "viewer": ParameterValue(LaunchConfiguration("viewer"), value_type=bool),
                        "real_time": ParameterValue(
                            LaunchConfiguration("real_time"), value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package="xbox_send",
                executable="xbox_send_node",
                name="xbox_send_node",
                output="screen",
            ),
            # 手柄驱动：读 /dev/input/js* 并发布 /joy，xbox_send_node 订阅它
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("joy")),
            ),
        ]
    )
