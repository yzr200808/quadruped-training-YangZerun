import os
from glob import glob

from setuptools import setup

package_name = "mit_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yzr",
    maintainer_email="yzr@todo.todo",
    description="MuJoCo 仿真控制节点：接收 MIT 参数，返回 q / dq / ddq / tau",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mit_sim_node = mit_sim.mit_sim_node:main",
        ],
    },
)
