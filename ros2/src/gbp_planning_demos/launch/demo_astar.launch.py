#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Dijkstra (비교 기준)
        Node(
            package='gbp_planning_demos',
            executable='simple_grid_planner',
            name='planner_dijkstra',
            parameters=[
                {'algorithm': 'dijkstra'},
                {'use_diagonal': True},
            ],
            remappings=[
                ('demo_path', 'planner_dijkstra/demo_path'),
            ],
            output='screen',
        ),

        # A* (Manhattan heuristic)
        Node(
            package='gbp_planning_demos',
            executable='simple_grid_planner',
            name='planner_astar',
            parameters=[
                {'algorithm': 'astar'},
                {'heuristic': 'octile'},   # manhattan / octile / euclidean 중 선택
                {'use_diagonal': True},
            ],
            remappings=[
                ('demo_path', 'planner_astar/demo_path'),
            ],
            output='screen',
        ),
    ])
