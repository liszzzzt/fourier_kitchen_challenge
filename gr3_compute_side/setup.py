from setuptools import find_namespace_packages, setup

setup(
    name="gr3_compute_side",
    version="0.1.0",
    package_dir={"": "src"},
    packages=find_namespace_packages(where="src"),
    entry_points={
        "console_scripts": [
            "start_server=gr3_compute_side.main:main",
        ],
    },
)