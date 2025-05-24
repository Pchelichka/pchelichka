from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera = Node(
        package='perception',
        executable='camera'
    )
    telemetry = Node(
        package='perception',
        executable='telemetry'
    )

    return LaunchDescription([
        camera,
        telemetry
    ])
