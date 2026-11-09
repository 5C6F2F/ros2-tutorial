from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_dir = PathJoinSubstitution([FindPackageShare("omni_bringup"), "config"])
    robot_config = PathJoinSubstitution([config_dir, "robot.yaml"])
    hardware_config = PathJoinSubstitution([config_dir, "fake_hardware.yaml"])
    localization_config = PathJoinSubstitution([config_dir, "localization.yaml"])
    scenario_config = PathJoinSubstitution(
        [config_dir, "scenario", [LaunchConfiguration("scenario"), ".yaml"]]
    )
    qos_config = PathJoinSubstitution(
        [config_dir, "qos", [LaunchConfiguration("qos"), ".yaml"]]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("omni_bringup"), "rviz", LaunchConfiguration("rviz_config")]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_fake_mcu",
                default_value="true",
                description="fake_mcuを起動する。bag再生時はfalseにする",
            ),
            DeclareLaunchArgument(
                "use_rviz", default_value="true", description="RViz2を起動する"
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="/clockの時刻で動かす。bag再生時はtrueにする",
            ),
            DeclareLaunchArgument(
                "scenario",
                default_value="ideal",
                description="config/scenario/ から重ねるYAML",
            ),
            DeclareLaunchArgument(
                "qos", default_value="default", description="config/qos/ から重ねるYAML"
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="main.rviz",
                description="rviz/ から読み込む設定",
            ),
            SetParameter(
                name="use_sim_time", value=LaunchConfiguration("use_sim_time")
            ),
            # 後に書いたYAMLの値が優先される
            Node(
                package="omni_fake_hardware",
                executable="fake_mcu",
                parameters=[robot_config, hardware_config, scenario_config, qos_config],
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_fake_mcu")),
            ),
            Node(
                package="omni_localization",
                executable="encoder_odometry",
                parameters=[
                    robot_config,
                    localization_config,
                    scenario_config,
                    qos_config,
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
