import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

    holo_zed_prefix = get_package_share_directory('zed_wrapper')
    start_holo_zed_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(holo_zed_prefix, 'launch', 'zed2i.launch.py'))
    )

    base_to_zed_publisher = ExecuteProcess(
        cmd=["ros2", "run", "tf2_ros", "static_transform_publisher", '0', '0', '0.5', '0', '0', '0', "base_link", "zed2i_base_link"], output="screen"
    )

    aruco_node = ExecuteProcess(
        cmd=["ros2", "run", "k_algorithm", "k_algorithm_node.py"]
    )
    

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', '/home/woojin/holo_ws/src/k_algorithm/rviz/k_algorithm.rviz'],
        ),
        start_holo_zed_cmd,
        base_to_zed_publisher,
        aruco_node,
    ])

    
