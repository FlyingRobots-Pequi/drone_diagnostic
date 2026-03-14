from glob import glob
from setuptools import find_packages, setup

package_name = "drone_diagnostics"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "diagnostic_node = drone_diagnostics.diagnostic_node:main",
            "diagnostics_logger = drone_diagnostics.diagnostics_logger_node:main",
        ],
    },
)
