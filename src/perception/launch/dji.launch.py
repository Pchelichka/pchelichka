from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessStart
from launch.events import matches_action

def generate_launch_description():
    dji_node = Node(
        package='dji',
        executable='dji',
    )

    camera_node = Node(
        package='perception',
        executable='camera',
		arguments=['--mode', 'digital']
    )

    launch_camera_node = RegisterEventHandler(
        OnProcessStart(
            target_action=dji_node,
            on_start=[camera_node]
        )
    )

    return LaunchDescription([
        dji_node,
        launch_camera_node
    ])