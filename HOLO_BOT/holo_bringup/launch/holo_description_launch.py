import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # scout mini package
    scout_prefix = get_package_share_directory('scout_base')
    start_scout_mini_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(scout_prefix, 'launch', 'scout_mini_base.launch.py'))
    )

    # rplidar package
    s2lidar_prefix = get_package_share_directory('rplidar_ros')
    start_s2lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(s2lidar_prefix, 'launch', 'rplidar_s2_launch.py'))
    )

    # holo_gps package
    holo_gps_prefix = get_package_share_directory('holo_gps_rtk')
    start_holo_gps_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(holo_gps_prefix, 'launch', 'holo_gps.launch.py'))
    )

    # c920 package
    usb_cam_prefix = get_package_share_directory('usb_cam')
    start_usb_cam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(usb_cam_prefix, 'launch', 'usb_cam_launch.py'))
    )

    zed_prefix = get_package_share_directory('zed_wrapper')
    start_zed_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(zed_prefix, 'launch', 'zed2i.launch.py'))
    )    

    # um7 package
    start_um7_cmd = ExecuteProcess(
        cmd=["ros2", "run", "umx_driver", "um7_driver"], output="screen"
    )

    # imu_complementary_filter package
    imu_complementary_filter_prefix = get_package_share_directory('imu_complementary_filter')
    start_imu_complementary_filter_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(imu_complementary_filter_prefix, 'launch', 'complementary_filter.launch.py'))
    )

    # Robot TF publisher
    base_to_laser_publisher = ExecuteProcess(
        cmd=["ros2", "run", "tf2_ros", "static_transform_publisher", '0', '0', '0', '-3.14159', '0', '0', "base_link", "laser_link"], output="screen"
    )

    base_to_gps_publisher = ExecuteProcess(
        cmd=["ros2", "run", "tf2_ros", "static_transform_publisher", '0', '0', '0', '0', '0', '0', "base_link", "gps_link"], output="screen"
    )

    base_to_imu_publisher = ExecuteProcess(
        cmd=["ros2", "run", "tf2_ros", "static_transform_publisher", '0', '0', '0', '0', '0', '0', "base_link", "imu_link"], output="screen"
    )

    base_to_zed_publisher = ExecuteProcess(
        cmd=["ros2", "run", "tf2_ros", "static_transform_publisher", '0', '0', '0.5', '0', '0', '0', "base_link", "zed2i_base_link"], output="screen"
    )

    return LaunchDescription([
        # Node(
        #         package='imu_complementary_filter',
        #         executable='complementary_filter_node',
        #         name='complementary_filter_gain_node',
        #         output='screen',
        #         parameters=[
        #             {'do_bias_estimation': True},
        #             {'do_adaptive_gain': True},
        #             {'use_mag': False},
        #             {'gain_acc': 0.01},
        #             {'gain_mag': 0.01},
        #         ],
        #         remappings=[
        #         ('imu/data_raw', 'zed2i/zed_node/imu/data_raw'),
        #         ('imu/data', 'zed2i/zed_node/imu/data_com'),
        #         ],
        #     ),
        start_scout_mini_cmd,
        start_s2lidar_cmd,
        # start_holo_gps_cmd,
        # start_usb_cam_cmd,
        # start_zed_cmd,
        start_um7_cmd,
        start_imu_complementary_filter_cmd,
        base_to_laser_publisher,
        base_to_gps_publisher,
        base_to_imu_publisher,
        base_to_zed_publisher,
    ])
