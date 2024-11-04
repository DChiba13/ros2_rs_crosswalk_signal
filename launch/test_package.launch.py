from os.path import join
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    launcher_description_list = []

    launch_file_infos = [
        ('ros2_rs_crosswalk_signal', 'launch/container.launch.py'),
        ('ros2_rs_crosswalk_signal', 'launch/test_img_pub.launch.py'),
        ('ros2_rs_crosswalk_signal', 'launch/crosswalk_signal.launch.py'),
        ('ros2_rs_crosswalk_signal', 'launch/test_img_sub.launch.py'),
        ('ros2_rs_crosswalk_signal', 'launch/test_light_sub.launch.py'),
        # ('ros2_rs_crosswalk_signal', 'launch/check_hsv.launch.py'),
        # ('ros2_rs_crosswalk_signal', 'launch/capture_cam_log.launch.py'),
        # ('ros2_rs_crosswalk_signal', 'launch/log_img_pub.launch.py'),

        # ('ros2_rs_launcher_2023', 'launch/rs_camera.launch.py'),
    ]

    # 各 launch ファイルを IncludeLaunchDescription で追加
    for launch_file_info in launch_file_infos:
        pkg_prefix = get_package_share_directory(launch_file_info[0])
        path = join(pkg_prefix, launch_file_info[1])
        launcher = IncludeLaunchDescription(PythonLaunchDescriptionSource(path))
        launcher_description_list.append(launcher)
        print(f'{launch_file_info[1]}: launch_check')

    print("all launch")
    return LaunchDescription(launcher_description_list)
