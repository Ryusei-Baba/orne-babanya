import os
import yaml
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # パラメータファイルのパス設定
    main_executor_dir = get_package_share_directory('main_executor')
    config_file_path = os.path.join(main_executor_dir, 'config', 'main_params.yaml')

    icart_mini_driver_dir = get_package_share_directory('icart_mini_driver')
    ypspur_param = os.path.join(icart_mini_driver_dir, 'config', 'box_v1.param')
    ypspur_coordinator_path = os.path.join(icart_mini_driver_dir, 'scripts', 'ypspur_coordinator_bridge')

    # 起動パラメータファイルのロード
    with open(config_file_path, 'r') as file:
        launch_params = yaml.safe_load(file)['launch']['ros__parameters']

    # メイン実行機ノードの作成
    main_exec_node = Node(
        package='main_executor',
        executable='main_exec',
        parameters=[config_file_path],
        output='screen'
    )

    # 操縦機ノードの作成
    joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config_file_path],
        output='screen'
    )

    # ypspur コーディネータープロセスの作成
    ypspur_coordinator_process = ExecuteProcess(
        cmd=[ypspur_coordinator_path, ypspur_param],
        shell=True
    )

    # LaunchDescription の作成
    launch_description = LaunchDescription()

    # 条件に応じてノードを追加
    if launch_params.get('joy', False):
        launch_description.add_action(joy_node)

    launch_description.add_action(main_exec_node)
    launch_description.add_action(ypspur_coordinator_process)

    return launch_description