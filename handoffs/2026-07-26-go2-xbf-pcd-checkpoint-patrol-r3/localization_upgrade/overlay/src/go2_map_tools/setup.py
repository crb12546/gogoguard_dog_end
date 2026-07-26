from setuptools import find_packages, setup


package_name = "go2_map_tools"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[],
    zip_safe=True,
    maintainer="Go2 Patrol Team",
    maintainer_email="maintainer@example.com",
    description=(
        "Deterministic PCD processing, map tiling, and place descriptor tools "
        "for Go2 patrol."
    ),
    license="Apache-2.0",
    python_requires=">=3.8",
    entry_points={
        "console_scripts": [
            "go2-map = go2_map_tools.cli:main",
        ],
    },
)
