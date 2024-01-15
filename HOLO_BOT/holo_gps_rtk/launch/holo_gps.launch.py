import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # zed_f9p package
    zed_f9p_prefix = get_package_share_directory('ublox_gps')
    start_zed_f9p_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(zed_f9p_prefix, 'launch', 'ublox_gps_node-launch.py'))
    )

    # ntrip client package
    ntrip_prefix = get_package_share_directory('ntrip_client')
    start_ntrip_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ntrip_prefix, 'launch', 'ntrip_client_launch.py'))
    )

    return LaunchDescription([
        start_zed_f9p_cmd,
        start_ntrip_cmd,
    ])