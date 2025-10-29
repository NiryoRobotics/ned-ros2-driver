from setuptools import find_packages, setup
from glob import glob
import os

package_name = "niryo_ned_ros2_driver"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pyyaml", "roslibpy<2.0.0"],
    zip_safe=True,
    maintainer="Thomas Degallaix",
    maintainer_email="t.degallaix@niryo.com",
    description="ROS2 driver for Niryo's Ned robots",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ros2_driver = niryo_ned_ros2_driver.bridge_node:main",
        ],
    },
)
