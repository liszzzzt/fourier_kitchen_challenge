from setuptools import setup

setup(
    name="gr3_gripper",
    version="0.1.0",
    packages=["gr3_gripper"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/gr3_gripper"]),
        ("share/gr3_gripper", ["package.xml", "README.md"]),
        ("share/gr3_gripper/models", ["models/smoke_fixture.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Fourier kitchen challenge contributors",
    maintainer_email="",
    description="ROS2 transport for the existing simulation-only Robotiq controller",
    license="Unspecified",
    entry_points={"console_scripts": [
        "gripper_client=gr3_gripper.client:main",
        "gripper_demo=gr3_gripper.demo:main",
        "gripper_preflight=gr3_gripper.preflight:main",
    ]},
)
