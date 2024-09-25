from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera = Node(
        package='perception',
        executable='camera_analog'
    )
    telemetry = Node(
        package='perception',
        executable='telemetry'
    )

    return LaunchDescription([
        camera,
        telemetry
    ])
