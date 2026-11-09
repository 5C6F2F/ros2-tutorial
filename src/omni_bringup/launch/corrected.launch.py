from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("omni_bringup")
    hardware_config = PathJoinSubstitution([share, "config", "fake_hardware.yaml"])
    localization_config = PathJoinSubstitution([share, "config", "localization.yaml"])

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PathJoinSubstitution([share, "launch", "bringup.launch.py"]),
                launch_arguments={
                    "scenario": "corrected",
                    "rviz_config": "advanced.rviz",
                }.items(),
            ),
            Node(
                package="omni_fake_hardware",
                executable="fake_lidar_localizer",
                parameters=[hardware_config],
                output="screen",
            ),
            Node(
                package="omni_localization",
                executable="map_odom_corrector",
                parameters=[localization_config],
                output="screen",
            ),
        ]
    )
