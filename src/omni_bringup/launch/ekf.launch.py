from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('omni_bringup')
    hardware_config = PathJoinSubstitution([share, 'config', 'fake_hardware.yaml'])
    ekf_config = PathJoinSubstitution([share, 'config', 'ekf.yaml'])

    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([share, 'launch', 'bringup.launch.py']),
            launch_arguments={
                'scenario': 'ekf',
                'rviz_config': 'advanced.rviz',
            }.items(),
        ),
        Node(
            package='omni_fake_hardware',
            executable='fake_lidar_localizer',
            parameters=[hardware_config],
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            parameters=[ekf_config],
            remappings=[('odometry/filtered', 'odometry/local')],
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            parameters=[ekf_config],
            remappings=[('odometry/filtered', 'odometry/global')],
            output='screen',
        ),
    ])
