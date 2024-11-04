from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription

def generate_launch_description():
    pkg_prefix = get_package_share_directory('ros2_rs_crosswalk_signal')
    capture_cam_log = LoadComposableNodes(
        target_container='rs_container',  # コンテナ名はcontainer.launch.pyで指定
        composable_node_descriptions=[
            ComposableNode(
                package='ros2_rs_crosswalk_signal',
                plugin='crosswalk_signal::CaptureCamLog',
                name='capture_cam_log',
                remappings=[
                    ('/camera1/image', '/camera1/image'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ]
    )
    return LaunchDescription([capture_cam_log])