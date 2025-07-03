from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.event_handlers.on_process_exit import OnProcessExit
from launch_ros.actions import Node
import os


def get_controller_dir():
    return get_package_share_directory('controller')

def get_perception_dir():
    return get_package_share_directory('perception')

def generate_launch_description():
    elrs = Node(
        package='elrs',
        executable='elrs_node',
    )

    delete_folder = ExecuteProcess(cmd=['rm', '-Rf', 'streaming'],
                                    cwd=os.path.join('rosbags'))
    run_logging = ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', 'streaming', '/target', '/effort', '/velocity', '/velocity_target', '/acc', '/acc_target'],
                                    cwd=os.path.join('rosbags'))
    
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_perception_dir(),
                'launch',
                'dji.launch.py'
            ])
        )
    )

    return LaunchDescription([
		elrs,
        vision_launch,
        delete_folder,
        RegisterEventHandler(OnProcessExit(
            target_action=delete_folder,
            on_exit=run_logging
        )),
    ])
