from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # パッケージ名と使用ファイルの定義
    package_name = "simulator"
    xacro_file = "orne_box_mid360.urdf.xacro"
    rviz_config = "urdf.rviz"

    # xacro を使って URDF を生成
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(package_name), "urdf", xacro_file])
    ])
    robot_description = {"robot_description": robot_description_content}

    # RViz設定ファイルのパス
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare(package_name), "rviz", rviz_config]
    )

    # ロボットの状態をパブリッシュするノード
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ジョイント状態をGUIから操作するノード
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    # RViz起動ノード
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
    )

    # 実行するノードをまとめる
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
