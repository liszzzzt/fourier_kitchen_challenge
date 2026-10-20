from setuptools import find_namespace_packages, setup

setup(
    name="gr3_robot_side",
    version="0.1.0",
    package_dir={"": "src"},
    packages=find_namespace_packages(where="src"),
    entry_points={
        "console_scripts": [
            "start_gr3=gr3_robot_side.main:main",
        ],
    },
)