from setuptools import find_packages, setup

package_name = "picar_ros"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/picar.launch.py"]),
        ("share/" + package_name + "/config", ["config/params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="ROS 2 nodes (perception, planning, action) for the PiCar-4WD robot.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "perception_node = picar_ros.perception_node:main",
            "planning_node = picar_ros.planning_node:main",
            "action_node = picar_ros.action_node:main",
        ],
    },
)
