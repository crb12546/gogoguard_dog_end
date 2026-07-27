from glob import glob
from setuptools import find_packages, setup


package_name = "go2_checkpoint_patrol"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=[],
    zip_safe=True,
    maintainer="Go2 Patrol Team",
    maintainer_email="maintainer@example.com",
    description=(
        "Fail-closed checkpoint localization adapter around the existing "
        "Go2 CSV waypoint follower."
    ),
    license="Apache-2.0",
    python_requires=">=3.8",
    entry_points={
        "console_scripts": [
            (
                "checkpoint-localization-coordinator = "
                "go2_checkpoint_patrol.checkpoint_coordinator:main"
            ),
        ],
    },
)
