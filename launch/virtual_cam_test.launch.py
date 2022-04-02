import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    vcam_path = os.path.join(get_package_share_directory('rm_cam'), 'launch', 'virtual_cam.launch.py')
    auto_aim_path = os.path.join(get_package_share_directory('rm_auto_aim'), 'launch', 'auto_aim_node.launch.py')

    virtual_cam_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(vcam_path))
    auto_aim_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(auto_aim_path))

    ld = LaunchDescription()
    ld.add_action(virtual_cam_node)
    ld.add_action(auto_aim_node)

    return ld
