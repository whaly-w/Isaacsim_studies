from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sub_topic = LaunchConfiguration('sub_topic')
    return LaunchDescription([
        DeclareLaunchArgument(
            'sub_topic',
            default_value= '/topic'
            ),
        
        Node(
            package="test_pkg",
            executable="talker",
            name="talker_node",
            ),
        Node(
            package="test_pkg",
            executable="subPub",
            name="new_listener_node",
            ),
        Node(
            package= 'test_pkg',
            executable= 'listener',
            name= 'listener',
            parameters= [
                {'sub_topic': sub_topic}
            ]
        )
    ])