from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('search_and_nav')

    objects_path = os.path.join(pkg_share, 'config')
    mission_config = os.path.join(pkg_share, 'config', 'mission.yaml')
    exploration_config = os.path.join(pkg_share, 'config', 'exploration.yaml')
    hazard_config = os.path.join(pkg_share, 'config', 'hazard.yaml')
    path_config = os.path.join(pkg_share, 'config', 'path.yaml')

    find_obj_node = Node(
        package='find_object_2d',
        executable='find_object_2d',
        parameters=[{
            'objects_path': objects_path,
            'gui': False, # Set to False to save RAM/CPU
            'subscribe_depth': True
        }],
        remappings=[
            ('image', '/oak/rgb/image_raw') # Match your robot's camera topic
        ],
        output='screen'
    )

    return LaunchDescription([
        find_obj_node,
        Node(
            package='search_and_nav',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[mission_config]
        ),
        Node(
            package='search_and_nav',
            executable='exploration_node',
            name='exploration_node',
            output='screen',
            parameters=[exploration_config]
        ),
        Node(
            package='search_and_nav',
            executable='hazard_mapper',
            name='hazard_mapper',
            output='screen',
            parameters=[hazard_config]
        ),
        Node(
            package='search_and_nav',
            executable='path_tracker',
            name='path_tracker',
            output='screen',
            parameters=[path_config]
        ),
        Node(
            package='search_and_nav',
            executable='start_detector',
            name='start_detector',
            output='screen',
            parameters=[mission_config]
        ),
        Node(
            package='search_and_nav',
            executable='mock_detection_adapter',
            name='mock_detection_adapter',
            output='screen'
        ),
    ])