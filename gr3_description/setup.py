import os
from glob import glob

from setuptools import find_packages, setup


package_name = "gr3_description"


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
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hongkun",
    maintainer_email="yuhongkun231@gmail.com",
    description="GR3 robot description and robot_state_publisher launcher.",
    license="Apache-2.0",
)
