from setuptools import find_namespace_packages, setup

setup(
    name="gr3_compute_side",
    version="0.1.0",
    package_dir={"": "src"},
    packages=find_namespace_packages(where="src"),
    package_data={"gr3_compute_side.vision.config": ["*.yaml"],
                  "gr3_compute_side.manipulation": ["model/*.xml"]},   # 右臂逆解用的 GR3_V233 运动学模型
    entry_points={
        "console_scripts": [
            "start_server=gr3_compute_side.main:main",
        ],
    },
)