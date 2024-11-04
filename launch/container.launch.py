from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    container = Node(
        package='rclcpp_components',
        executable='component_container',
        name='rs_container',
        output='screen'
    )
    return LaunchDescription([
      container
    ])
