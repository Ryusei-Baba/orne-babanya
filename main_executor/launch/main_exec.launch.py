import os
import yaml
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution


def generate_launch_description():
    # パラメータファイルのパス設定
    main_executor_dir = get_package_share_directory('main_executor')
    config_file_dir = os.path.join(main_executor_dir, 'config')
    config_file_path = os.path.join(config_file_dir, 'main_params.yaml')

    icart_mini_driver_dir = get_package_share_directory('icart_mini_driver')
    icart_mini_driver_config_dir = os.path.join(icart_mini_driver_dir, 'config')
    ypspur_param = os.path.join(icart_mini_driver_config_dir, 'box_v1.param')
    ypspur_coordinator_path = os.path.join(icart_mini_driver_dir, 'scripts', 'ypspur_coordinator_bridge')

    # robot_description_dir = get_package_share_directory('robot_description')
    # robot_description_config_dir = os.path.join(robot_description_dir, 'urdf')
    # robot_description_path = os.path.join(robot_description_config_dir, 'orne_box_mid360.urdf.xacro')

    rviz_config_path = os.path.join(config_file_dir, 'display.rviz')

    # 起動パラメータファイルのロード
    with open(config_file_path, 'r') as file:
        launch_params = yaml.safe_load(file)['launch']['ros__parameters']

    # ypspur コーディネータープロセスの作成
    ypspur_coordinator_process = ExecuteProcess(
        cmd=[ypspur_coordinator_path, ypspur_param],
        shell=True
    )

    # メイン実行機ノードの作成
    main_exec_node = Node(
        package='main_executor',
        executable='main_exec',
        parameters=[config_file_path],
        output='screen'
    )

    # joyノードの作成
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[config_file_path],
        output='screen'
    )

    # 操縦機ノードの作成
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config_file_path],
        output='screen'
    )



    # urg_nodeの作成
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[config_file_path],
        remappings=[('/scan', '/scan')],
        output='screen'
    )

    # usb_camノードの作成
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node',
        parameters=[config_file_path],
        remappings=[('/image_raw', '/image_raw'),
                    ('/camera_info', '/camera_info')],
        output='screen'
    )

    # RVizノードの作成
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_config_path],
        output='screen'
    )

    # # robot_localizationノードの作成
    # robot_localization_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     parameters=[config_file_path],
    #     remappings=[('/odometry/filtered', '/odometry/filtered')],
    #     output='screen'
    # )

    # # robot_state_publisherノードの作成
    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     parameters=[{"robot_description": Command([
    #         PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', robot_description_path
    #     ])}],
    #     remappings=[('/tf', '/tf'),
    #                 ('/tf_static', '/tf_static')],
    #     output='screen'
    # )

    # joint_state_publisherノードの作成
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        remappings=[('/joint_states', '/joint_states')],
        output='screen'
    )

    # LaunchDescription の作成
    launch_description = LaunchDescription()

    # 条件に応じてノードを追加
    if launch_params.get('joy', False):
        launch_description.add_action(joy_node)
    if launch_params.get('rviz', False):
        launch_description.add_action(rviz_node)
    if launch_params.get('urg', False):
        launch_description.add_action(urg_node)
    if launch_params.get('usb_cam', False):
        launch_description.add_action(usb_cam_node)

    launch_description.add_action(ypspur_coordinator_process)
    launch_description.add_action(main_exec_node)
    # launch_description.add_action(teleop_node)
    # launch_description.add_action(robot_localization_node)
    # launch_description.add_action(robot_state_publisher_node)
    # launch_description.add_action(joint_state_publisher_node)

    return launch_description