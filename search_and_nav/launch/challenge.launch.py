import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Dynamically find the path where the markers are installed
    pkg_share = get_package_share_directory('search_and_nav')
    markers_path = os.path.join(pkg_share, 'config')

    return LaunchDescription([
        # Vision Node: Required to identify the 13 markers
        Node(
            package='find_object_2d',
            executable='find_object_2d',
            name='find_object_2d',
            output='screen',
            parameters=[{
                'objects_path': markers_path,
                'gui': False, # Recommended for the real-robot eval
                'subscribe_depth': True
            }],
            remappings=[
                # Use the topic that matches the camera on the ROSBot
                ('image', '/oak/rgb/color') 
            ]
        ),

        Node(
            package='search_and_nav',
            executable='mission_node',
            output='screen'
        ),

        Node(
            package='search_and_nav',
            executable='navigation_node',
            output='screen'
        ),

        Node(
            package='search_and_nav',
            executable='perception_node',
            output='screen'
        ),
    ])