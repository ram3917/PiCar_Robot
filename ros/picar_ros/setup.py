from setuptools import find_packages, setup

package_name = "picar_ros"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/picar.launch.py"]),
        ("share/" + package_name + "/config", ["config/params.yaml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    description="ROS 2 controller and motion planner nodes for the PiCar-4WD robot.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "controller_node = picar_ros.controller_node:main",
            "motion_planner_node = picar_ros.motion_planner_node:main",
        ],
    },
)
