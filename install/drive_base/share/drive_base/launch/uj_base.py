from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="drive_base",
            executable="drive_base",
            name="drive_base",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"serial_port": "/dev/ttyACM0"},
                {"baud": 38400},
                {"wheel_diameter": 120.0},
                {"encoder_ticks": 537.7},
                {"max_velocity": 1800.0}
            ]
        )
    ])