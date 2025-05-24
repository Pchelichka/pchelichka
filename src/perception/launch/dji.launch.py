from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Full path to the script inside the share directory
    script_path = os.path.join(
        get_package_share_directory('perception'),
        'scripts',
        'dji_stream'  # Your executable script
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=[f'{script_path} | ros2 run perception camera -- --mode digital'],
            shell=True,
            output='screen'
        )
    ])
