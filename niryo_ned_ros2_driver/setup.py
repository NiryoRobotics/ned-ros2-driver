from setuptools import find_packages, setup

package_name = "niryo_ned_ros2_driver"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Thomas Degallaix",
    maintainer_email="t.degallaix@niryo.com",
    description="ROS2 driver for Niryo's Ned robots",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
