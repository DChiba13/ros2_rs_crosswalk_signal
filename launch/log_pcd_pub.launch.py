from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription

def generate_launch_description():
    pkg_prefix = get_package_share_directory('ros2_rs_crosswalk_signal')

    log_pcd_pub = LoadComposableNodes(
        target_container='rs_container',
        composable_node_descriptions=[
            ComposableNode(
                package='ros2_rs_crosswalk_signal',
                plugin='crosswalk_signal::LogPcdPublisher',
                name='log_pcd_pub',
                parameters=[join(pkg_prefix, 'cfg/log_pcd_pub.yaml')],
                remappings=[
                    ('/lidar/points', '/lidar/points'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ]
    )

    return LaunchDescription([log_pcd_pub])
