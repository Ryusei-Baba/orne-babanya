import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # -------------------------------------
    # パス設定
    # -------------------------------------
    simulator_dir = get_package_share_directory('simulator')
    # world_file = os.path.join(simulator_dir, 'worlds', 'tsudanuma2-3.sdf')
    world_file = os.path.join(simulator_dir, 'worlds', 'orne_box_factory.sdf')
    param_file = os.path.join(simulator_dir, 'config', 'sim_params.yaml')

    # -------------------------------------
    # パラメータファイルの読み込みと分離
    # -------------------------------------
    with open(param_file, 'r') as file:
        full_params = yaml.safe_load(file)
        joy_params = full_params.get('joy_node', {}).get('ros__parameters', {})
        teleop_params = full_params.get('teleop_twist_joy_node', {}).get('ros__parameters', {})

    # -------------------------------------
    # Gazebo起動（指定ワールドで起動）
    # -------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments=[('gz_args', f'-r -v 4 {world_file}')]
    )

    # -------------------------------------
    # GZ ⇔ ROS2 ブリッジ
    # -------------------------------------
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            # '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            # '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            # '/depth_camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            # '/depth_camera/image_raw/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            # '/navsat@sensor_msgs/msg/NavSatFix@ignition.msgs.NavSat',
            '/livox/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/livox/lidar/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
        ],
        output='screen'
    )

    # -------------------------------------
    # Joy ノード
    # -------------------------------------
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[joy_params],
    )

    # -------------------------------------
    # teleop_twist_joy ノード
    # -------------------------------------
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[teleop_params],
    )

    # -------------------------------------
    # LaunchDescription を返す
    # -------------------------------------
    return LaunchDescription([
        gazebo,
        bridge_node,
        joy_node,
        teleop_node,
    ])
