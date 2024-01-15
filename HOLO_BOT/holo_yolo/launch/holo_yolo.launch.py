import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

    holo_yolo = ExecuteProcess(
        cmd=["ros2", "run", "holo_yolo", "detector.py"]
    )
    

    return LaunchDescription([
        holo_yolo,
    ])

    
