from glob import glob
import os

from setuptools import find_packages, setup

package_name = "g1_xr_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={
        "g1_xr_teleop": [
            "assets/g1/*",
            "assets/g1/meshes/*",
        ],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "LOG_zh-CN.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy", "scipy", "pin>=3.3,<4"],
    extras_require={"test": ["pytest"]},
    zip_safe=False,
    maintainer="asanolab",
    maintainer_email="todo@example.com",
    description="ROS 2 XR teleoperation bridge for the Unitree G1 control interface.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "xr_teleop_node = g1_xr_teleop.xr_teleop_node:main",
            "ik_offline_check = g1_xr_teleop.ik_offline_check:main",
        ],
    },
)
