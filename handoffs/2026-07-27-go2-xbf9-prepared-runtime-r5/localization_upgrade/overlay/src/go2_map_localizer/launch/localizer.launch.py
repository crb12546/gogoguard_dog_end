# Copyright 2026 Go2 Robotics Team
# SPDX-License-Identifier: Apache-2.0
"""Launch the production Go2 MID-360 map localizer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    default_parameters = str(
        Path(get_package_share_directory("go2_map_localizer"))
        / "config"
        / "localizer.yaml"
    )
    map_manifest = LaunchConfiguration("map_manifest")
    parameters_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_manifest",
                description="Absolute path to the verified map manifest.json",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_parameters,
                description="Localizer parameter YAML",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="go2_map_localizer",
                executable="go2_map_localizer_node",
                name="go2_map_localizer",
                output="screen",
                emulate_tty=True,
                parameters=[
                    parameters_file,
                    {
                        "map_manifest_path": map_manifest,
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
        ]
    )
