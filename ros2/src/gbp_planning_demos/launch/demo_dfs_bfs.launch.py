#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # BFS 경로
        Node(
            package='gbp_planning_demos',
            executable='simple_grid_planner',
            name='planner_bfs',
            parameters=[
                {'algorithm': 'bfs'},
                {'use_diagonal': False},
            ],
            remappings=[
                # 노드 내부에서는 'demo_path'로 퍼블리시하지만,
                # 실제 토픽 이름은 /planner_bfs/demo_path 로 바꿔준다.
                ('demo_path', 'planner_bfs/demo_path'),
            ],
            output='screen',
        ),

        # DFS 경로
        Node(
            package='gbp_planning_demos',
            executable='simple_grid_planner',
            name='planner_dfs',
            parameters=[
                {'algorithm': 'dfs'},
                {'use_diagonal': False},
            ],
            remappings=[
                ('demo_path', 'planner_dfs/demo_path'),
            ],
            output='screen',
        ),
    ])
