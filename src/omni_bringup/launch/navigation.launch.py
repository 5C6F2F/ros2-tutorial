from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_dir = PathJoinSubstitution([FindPackageShare("omni_bringup"), "config"])
    navigation_config = PathJoinSubstitution([config_dir, "navigation.yaml"])
    scenario_config = PathJoinSubstitution(
        [config_dir, "scenario", [LaunchConfiguration("scenario"), ".yaml"]]
    )
    qos_config = PathJoinSubstitution(
        [config_dir, "qos", [LaunchConfiguration("qos"), ".yaml"]]
    )
    parameters = [navigation_config, scenario_config, qos_config]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario",
                default_value="ideal",
                description="config/scenario/ から重ねるYAML",
            ),
            DeclareLaunchArgument(
                "qos", default_value="default", description="config/qos/ から重ねるYAML"
            ),
            Node(
                package="omni_navigation",
                executable="path_generator",
                parameters=parameters,
                output="screen",
            ),
            Node(
                package="omni_navigation",
                executable="path_follower",
                parameters=parameters,
                output="screen",
            ),
            Node(
                package="omni_navigation",
                executable="navigation_manager",
                parameters=parameters,
                output="screen",
            ),
        ]
    )
