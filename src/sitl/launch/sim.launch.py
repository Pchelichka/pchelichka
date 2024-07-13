from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers.on_process_io import OnProcessIO
from launch_ros.actions import Node
import os
import time


def get_betaflight_dir():
    return get_package_share_directory('sitl')

def get_betaflight_gazebo_dir():
    return get_package_share_directory('betaflight_gazebo')

"""
run_virtual_tty = ExecuteProcess(cmd=["socat", "-dd", "pty,link=/tmp/ttyS0,raw,echo=0", "tcp:127.0.0.1:5761"]),


def _run_virtual_tty_check(event):
    Consider betaflight_controller ready when 'bind port 5761 for UART1...' string is printed.

    Launches betaflight_controller node if ready.
    target_str = 'bind port 5761 for UART1'
    if target_str in event.text.decode():
        time.sleep(2)
        return run_virtual_tty
"""

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='If true, use simulated clock')

    os.environ["GZ_SIM_RESOURCE_PATH"] = os.path.join(get_betaflight_dir(), "models")
    os.environ["GZ_SIM_RESOURCE_PATH"] += ":" + os.path.join(get_betaflight_dir(), "worlds")
    print("os.environ[GZ_SIM_RESOURCE_PATH]", os.environ["GZ_SIM_RESOURCE_PATH"])
    world_name = LaunchConfiguration('world_name', default='empty_betaflight_world.sdf')
    world_name_arg = DeclareLaunchArgument('world_name',
                                           default_value=world_name,
                                           description='World name')

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/iris_with_Betaflight/model/iris_with_standoffs/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                   '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        remappings=[("/model/iris_with_Betaflight/model/iris_with_standoffs/pose", "/tf")],
        output='screen'
    )

    run_betaflight_sitl = ExecuteProcess(cmd=['betaflight_SITL.elf', "127.0.0.1"],
                                         cwd=os.path.join(get_betaflight_dir(), "config"),
                                         output='screen')
    # gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]), launch_arguments=[('gz_args', [' -r -v 4 ', LaunchConfiguration('world_name'), ".sdf"])])
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v 4', LaunchConfiguration('world_name')], 
        additional_env={
        'WAYLAND_DISPLAY': '',
        "GZ_SIM_SYSTEM_PLUGIN_PATH": os.pathsep.join(
            [
                os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", default=""),
                os.environ.get("LD_LIBRARY_PATH", default=""),
                os.path.join(get_betaflight_gazebo_dir(), 'lib'),
            ]
        )},
        output='screen'
    )
    return LaunchDescription([
        world_name_arg,
        use_sim_time_arg,
        gz_bridge,
        gazebo,
        RegisterEventHandler(OnProcessIO(
            target_action=gazebo,
            on_stderr=lambda event: run_betaflight_sitl if event.text.decode().strip() == 'BetaflightPlugin loaded.' else None # run_betaflight_sitl if event.text.decode().strip() == "" else None
        ))
    ])
