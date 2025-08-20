from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription

def generate_launch_description():
    pkg_prefix = get_package_share_directory('ros2_rs_crosswalk_signal')
    crosswalk_signal = LoadComposableNodes(
        target_container='rs_container',
        composable_node_descriptions=[
            ComposableNode(
                package='ros2_rs_crosswalk_signal',
                plugin='crosswalk_signal::Recognition',
                name='crosswalk_signal',
                # parameters=[join(pkg_prefix, 'cfg/crosswalk_signal_parameters.yaml')],
                remappings=[
                    ('/camera1/image', '/camera1/image'),
                    ('/lidar/points', '/pandar40/points'), 
                    ('/light_msg', '/light_msg'),
                    ('/signal_image', '/signal_image'),
                    ('/traffic_light/range_img', '/rs_points_processor/depth_img'),
                    ('/traffic_light/ref_img', '/rs_points_processor/ref_img'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ]
    )
    return LaunchDescription([crosswalk_signal])
