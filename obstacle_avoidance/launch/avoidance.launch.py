import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('obstacle_avoidance')
    params = os.path.join(share, 'config', 'world.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('autonomous', default_value='true',
                              description='false = drive yourself with teleop_twist_keyboard'),
        Node(package='obstacle_avoidance', executable='world_sim', parameters=[params], output='screen'),
        Node(package='obstacle_avoidance', executable='avoider', parameters=[params], output='screen',
             condition=IfCondition(LaunchConfiguration('autonomous'))),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'avoidance.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
