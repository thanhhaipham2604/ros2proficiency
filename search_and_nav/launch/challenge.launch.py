from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

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