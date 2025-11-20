#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('gbp_common')

    default_map = os.path.join(pkg_share, 'maps', 'simple_room.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'lecture1_grid.rviz')
    default_world = os.path.join(pkg_share, 'worlds', 'simple_room.world')

    map_file = LaunchConfiguration('map')
    rviz_file = LaunchConfiguration('rviz')
    world_file = LaunchConfiguration('world')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_file
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to the map yaml file.'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value=default_rviz,
            description='Full path to the RViz config file.'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Full path to the Gazebo world file.'
        ),

        gazebo_launch,

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            output='screen'
        ),
    ])
