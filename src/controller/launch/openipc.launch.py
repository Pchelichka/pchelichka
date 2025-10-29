from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.event_handlers.on_process_exit import OnProcessExit
from launch_ros.actions import Node
import os
import time


def get_controller_dir():
    return get_package_share_directory('controller')

def get_perception_dir():
    return get_package_share_directory('perception')

def generate_launch_description(): 
    run_logging = ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', f'topics-{time.time()}', '/target', '/effort', '/velocity', '/velocity_target', '/acc', '/acc_target'],
                                    cwd=os.path.join('rosbags'))
     
    camera_node = Node(
        package='perception',
        executable='camera',
		arguments=['--source', 'openipc']
    ) 
    controller_node = Node(
        package='controller',
        executable='absolute_controller',
		arguments=['--mode', 'elrs']
    ) 
    joystick_node = Node(
        package='controller',
        executable='joystick',
		arguments=['--ros-args', '--log-level', 'ERROR']
    )

    return LaunchDescription([
        camera_node,
		joystick_node,
		# controller_node,
		run_logging,
    ])
