#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # BFS (비교용)
        Node(
            package='gbp_planning_demos',
            executable='simple_grid_planner',
            name='planner_bfs',
            parameters=[
                {'algorithm': 'bfs'},
                {'use_diagonal': False},
            ],
            remappings=[
                ('demo_path', 'planner_bfs/demo_path'),
            ],
            output='screen',
        ),

        # Dijkstra
        Node(
            package='gbp_planning_demos',
            executable='simple_grid_planner',
            name='planner_dijkstra',
            parameters=[
                {'algorithm': 'dijkstra'},
                {'use_diagonal': False},
            ],
            remappings=[
                ('demo_path', 'planner_dijkstra/demo_path'),
            ],
            output='screen',
        ),
    ])
