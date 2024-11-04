from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_prefix = get_package_share_directory('ros2_rs_crosswalk_signal') 
    test_light_sub = Node(
        package='ros2_rs_crosswalk_signal', 
        executable='test_light_sub',        # ノードの実行可能ファイル名に変更
        name='test_light_sub',
        parameters=[join(pkg_prefix, 'cfg/your_parameter_file.yaml')],  # 必要なパラメータファイルを指定
        remappings=[
                    ('/light_msg', '/light/msg'),
                ],
        output='screen'  # ログを標準出力に表示
    )

    return LaunchDescription([test_light_sub])
