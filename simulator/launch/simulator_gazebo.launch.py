import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # -------------------------------------
    # パス設定
    # -------------------------------------
    simulator_dir = get_package_share_directory('simulator')
    world_file = os.path.join(simulator_dir, 'world', 'tsudanuma2-3.sdf')
    xacro_file = os.path.join(simulator_dir, 'urdf', 'orne_box_mid360_gazebo.urdf.xacro')
    param_file = os.path.join(simulator_dir, 'config', 'sim_params.yaml')

    # -------------------------------------
    # xacroからURDFを生成
    # -------------------------------------
    robot_description_config = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})
    robot_description = robot_description_config.toprettyxml(indent='  ')

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
    # パラメータファイルの読み込みと分離
    # -------------------------------------
    with open(param_file, 'r') as file:
        full_params = yaml.safe_load(file)
        joy_params = full_params.get('joy_node', {}).get('ros__parameters', {})
        teleop_params = full_params.get('teleop_twist_joy_node', {}).get('ros__parameters', {})

    # -------------------------------------
    # Robot State Publisher ノード
    # -------------------------------------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # -------------------------------------
    # Gazebo内にロボットをスポーン
    # -------------------------------------
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_description,
            '-name', 'orne_box',
            '-allow_renaming', 'false',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',
            '-Y', '0.0',
            '-R', '0.0',
            '-P', '0.0',
        ]
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
        robot_state_publisher_node,
        spawn_robot_node,
        bridge_node,
        joy_node,
        teleop_node,
    ])
