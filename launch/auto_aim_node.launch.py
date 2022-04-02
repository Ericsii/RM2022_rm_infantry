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

    param_path = os.path.join(get_package_share_directory("rm_auto_aim"), "config/rm_auto_aim.yaml")
    robot_name = 'infantry4'
    auto_aim_node = Node(
        package="rm_auto_aim",
        executable="auto_aim_node",
        namespace = robot_name,
        name="auto_aim_node",
        parameters=[param_path],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(auto_aim_node)

    return ld