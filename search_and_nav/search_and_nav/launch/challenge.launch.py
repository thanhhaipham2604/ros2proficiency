from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('par_snc')

    mission_config = os.path.join(pkg_share, 'config', 'mission.yaml')
    exploration_config = os.path.join(pkg_share, 'config', 'exploration.yaml')
    hazard_config = os.path.join(pkg_share, 'config', 'hazard.yaml')
    path_config = os.path.join(pkg_share, 'config', 'path.yaml')

    return LaunchDescription([
        Node(
            package='par_snc',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[mission_config]
        ),
        Node(
            package='par_snc',
            executable='exploration_node',
            name='exploration_node',
            output='screen',
            parameters=[exploration_config]
        ),
        Node(
            package='par_snc',
            executable='hazard_mapper',
            name='hazard_mapper',
            output='screen',
            parameters=[hazard_config]
        ),
        Node(
            package='par_snc',
            executable='path_tracker',
            name='path_tracker',
            output='screen',
            parameters=[path_config]
        ),
        Node(
            package='par_snc',
            executable='start_detector',
            name='start_detector',
            output='screen',
            parameters=[mission_config]
        ),
    ])