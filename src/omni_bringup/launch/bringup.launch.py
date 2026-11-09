from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_dir = PathJoinSubstitution([FindPackageShare("omni_bringup"), "launch"])

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PathJoinSubstitution([launch_dir, "sim.launch.py"])
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution([launch_dir, "navigation.launch.py"])
            ),
        ]
    )
