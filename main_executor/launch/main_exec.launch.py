def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
    from launch_ros.actions import Node
    from launch.event_handlers import OnProcessStart
    import os
    import yaml
    from ament_index_python.packages import get_package_share_directory

    # パラメータファイルのパス設定
    main_executor_dir = get_package_share_directory('main_executor')
    config_file_path = os.path.join(main_executor_dir, 'config', 'main_params.yaml')

    icart_mini_driver_dir = get_package_share_directory('icart_mini_driver')
    icart_mini_driver_config_dir = os.path.join(icart_mini_driver_dir, 'config')
    urdf_file = os.path.join(icart_mini_driver_config_dir, 'orne_box_mid360.urdf')
    ypspur_param = os.path.join(icart_mini_driver_config_dir, 'box_v1.param')
    ypspur_coordinator_path = os.path.join(icart_mini_driver_dir, 'scripts', 'ypspur_coordinator_bridge')

    # 起動パラメータファイルのロード
    with open(config_file_path, 'r') as file:
        launch_params = yaml.safe_load(file)['launch']['ros__parameters']

    # ypspur coordinator bridge 起動
    ypspur_coordinator_process = ExecuteProcess(cmd=[ypspur_coordinator_path, ypspur_param], shell=True, output='screen')

    # diff_driveハードウェア制御を使うros2_control_node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='ros2_control_node',
        parameters=[config_file_path, {'robot_description': open(urdf_file).read()}],
        output='screen'
    )
    spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 'diff_drive_controller', '--controller-manager', '/ros2_control_node'],
        parameters=[config_file_path],
        output='screen'
    )

    # joyノード
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[config_file_path],
        output='screen'
    )

    # teleop_twist_joyノード
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config_file_path],
        remappings=[('/cmd_vel', '/ros2_control_node/cmd_vel_unstamped')],
        output='screen'
    )

    # LaunchDescriptionに追加
    launch_description = LaunchDescription()

    if launch_params.get('joy', False):
        launch_description.add_action(joy_node)

    launch_description.add_action(ypspur_coordinator_process)
    launch_description.add_action(TimerAction(period=3.0,actions=[ros2_control_node]))
    launch_description.add_action(RegisterEventHandler(OnProcessStart(target_action=ros2_control_node,on_start=[spawner])))
    launch_description.add_action(teleop_node)

    return launch_description
