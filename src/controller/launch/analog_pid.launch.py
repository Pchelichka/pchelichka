from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers.on_process_exit import OnProcessExit
from launch_ros.actions import Node
import os


def get_controller_dir():
    return get_package_share_directory('controller')


def generate_launch_description():
    controller = Node(
        package='controller',
        executable='absolute_controller',
        additional_env={
        'PYTHONUNBUFFERED': '1',
		},
		output='screen'
    )

    delete_folder = ExecuteProcess(cmd=['rm', '-Rf', 'streaming'],
                                    cwd=os.path.join('rosbags'))
    run_logging = ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', 'streaming', '/target', '/effort', '/velocity', '/velocity_target', '/acc', '/acc_target'],
                                    cwd=os.path.join('rosbags'))

    return LaunchDescription([
        controller,
        delete_folder,
        RegisterEventHandler(OnProcessExit(
            target_action=delete_folder,
            on_exit=run_logging
        )),
    ])
