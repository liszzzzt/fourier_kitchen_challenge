import os
from glob import glob

from setuptools import find_packages, setup


package_name = "gr3_ros2_bringup"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (
            os.path.join("share", package_name, "config", "cartographer"),
            glob("config/cartographer/*.lua"),
        ),
        (
            os.path.join("share", package_name, "config", "nav2"),
            glob("config/nav2/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "config", "rviz2"),
            glob("config/rviz2/*.rviz"),
        ),
        (
            os.path.join("share", package_name, "maps", "kitchen"),
            glob("maps/kitchen/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hongkun",
    maintainer_email="yuhongkun231@gmail.com",
    description="ROS 2 launch and configuration package for the GR3 kitchen project.",
    license="Apache-2.0",
)
