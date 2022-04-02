import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    mvcam_path = os.path.join(get_package_share_directory('rm_entity_cam'), 'launch', 'mindvision_cam.launch.py')
    auto_aim_path = os.path.join(get_package_share_directory('rm_auto_aim'), 'launch', 'auto_aim_node.launch.py')

    mv_cam_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(mvcam_path))
    auto_aim_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(auto_aim_path))

    ld = LaunchDescription()
    ld.add_action(mv_cam_node)
    ld.add_action(auto_aim_node)

    return ld
