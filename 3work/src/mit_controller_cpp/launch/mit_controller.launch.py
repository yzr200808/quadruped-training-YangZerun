"""启动 MIT 参数控制器节点。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("mit_controller_cpp"), "config", "mit_controller.yaml"
    )

    controller_node = Node(
        package="mit_controller_cpp",
        executable="mit_controller_node",
        name="mit_controller_node",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([controller_node])
