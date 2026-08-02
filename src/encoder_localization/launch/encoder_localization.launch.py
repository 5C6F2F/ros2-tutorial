import math
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 測定輪
    ENCODER_RESOLUTION = 4096

    # 単位はmm
    TREAD_RADIUS = 186.5
    WHEEL_RADIUS = 30

    SQRT3 = math.sqrt(3)
    PI = math.pi

    # Front
    front_x = TREAD_RADIUS
    front_y = 0
    front_theta = PI / 2
    front_radius = WHEEL_RADIUS

    # Rear Left
    rear_left_x = -TREAD_RADIUS / 2.0
    rear_left_y = TREAD_RADIUS * SQRT3 / 2.0
    rear_left_theta = PI / 2 + PI * 2 / 3
    rear_left_radius = WHEEL_RADIUS

    # Rear Right
    rear_right_x = -TREAD_RADIUS / 2.0
    rear_right_y = -TREAD_RADIUS * SQRT3 / 2.0
    rear_right_theta = PI / 2 + PI * 4 / 3
    rear_right_radius = WHEEL_RADIUS

    # fmt: off
    wheel_config_list: list[float] = [
        front_x, front_y, front_theta, front_radius,
        rear_right_x, rear_right_y, rear_right_theta, rear_right_radius,
        rear_left_x, rear_left_y, rear_left_theta, rear_left_radius,
    ]
    # fmt: on

    FREQUENCY = 30

    return LaunchDescription(
        [
            Node(
                package="encoder_localization",
                executable="encoder_localization",
                name="encoder_localization",
                output="screen",
                parameters=[
                    {
                        "encoder_resolution": ENCODER_RESOLUTION,
                        "wheel_configs": wheel_config_list,
                        "frequency": FREQUENCY,
                    }
                ],
            ),
            Node(
                package="encoder_localization",
                executable="cmd_vel_to_encoder",
                name="cmd_vel_to_encoder",
                output="screen",
                parameters=[
                    {
                        "encoder_resolution": ENCODER_RESOLUTION,
                        "wheel_configs": wheel_config_list,
                        "frequency": FREQUENCY,
                    }
                ],
            ),
        ]
    )
