#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    start_row = 30
    start_col = 30
    goal_row  = 160
    goal_col  = 160

    common_params = [
        {'start_row': start_row},
        {'start_col': start_col},
        {'goal_row': goal_row},
        {'goal_col': goal_col},
        {'use_diagonal': False},     # connectivity
    ]

    return LaunchDescription([
        # DFS
        Node(
            package='gbp_planning_demos',
            executable='simple_grid_planner',
            name='planner_dfs',
            parameters=[
                {'algorithm': 'dfs'},
                *common_params,
            ],
            remappings=[
                ('demo_path', 'planner_dfs/demo_path'),
            ],
            output='screen',
        ),

        # BFS
        Node(
            package='gbp_planning_demos',
            executable='simple_grid_planner',
            name='planner_bfs',
            parameters=[
                {'algorithm': 'bfs'},
                *common_params,
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
                *common_params,
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
                {'heuristic': 'octile'},  
                *common_params,
            ],
            remappings=[
                ('demo_path', 'planner_astar/demo_path'),
            ],
            output='screen',
        ),
    ])
