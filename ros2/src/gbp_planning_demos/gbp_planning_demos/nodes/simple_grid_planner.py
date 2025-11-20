#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import List, Tuple, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

from gbp_planning_demos.algorithms.bfs import bfs_steps
from gbp_planning_demos.algorithms.dfs import dfs_steps
from gbp_planning_demos.algorithms.dijkstra import dijkstra_steps
from gbp_planning_demos.algorithms.astar import astar_steps
from gbp_planning_demos.utils.gridmap import GridMap

Coord = Tuple[int, int]


class SimpleGridPlanner(Node):

    def __init__(self):
        super().__init__('simple_grid_planner')

        # params
        self.declare_parameter('algorithm', 'bfs')          # bfs / dfs / dijkstra / astar
        self.declare_parameter('heuristic', 'manhattan')    # A* heuristic 
        self.declare_parameter('use_diagonal', True)        # connectivity
        self.declare_parameter('obstacle_threshold', 50)    # 
        self.declare_parameter('start_row', 10)            
        self.declare_parameter('start_col', 10)
        self.declare_parameter('goal_row', 190)
        self.declare_parameter('goal_col', 190)
        self.declare_parameter('frame_id', 'map')

        # QoS / sub / pub setting
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile=map_qos,
        )

        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.path_pub = self.create_publisher(
            Path,
            'demo_path',
            qos_profile=path_qos
        )

        self._map_received = False
        self.get_logger().info("SimpleGridPlanner node started. Waiting for /map ...")

    def map_callback(self, msg: OccupancyGrid):
        if self._map_received:
            return
        self._map_received = True

        self.get_logger().info("Received /map. Building GridMap and planning ...")

        gm = self._build_gridmap_from_occupancy(msg)

        self._set_start_goal(gm)

        if gm.start is None or gm.goal is None:
            self.get_logger().error(f"Start 또는 Goal 설정에 실패했습니다. start={gm.start}, goal={gm.goal}")
            return

        algo = self.get_parameter('algorithm').get_parameter_value().string_value
        heuristic = self.get_parameter('heuristic').get_parameter_value().string_value

        path_cells = self._run_algorithm(algo, heuristic, gm)

        if not path_cells:
            self.get_logger().warn(f"알고리즘 '{algo}'로 경로를 찾지 못했습니다.")
            return

        path_msg = self._build_path_msg(path_cells, msg)
        self.path_pub.publish(path_msg)
        self.get_logger().info(
            f"Published path (len={len(path_cells)}) using algorithm='{algo}'"
        )

    def _build_gridmap_from_occupancy(self, msg: OccupancyGrid) -> GridMap:
        H = msg.info.height
        W = msg.info.width
        data = np.array(msg.data, dtype=np.int16).reshape(H, W)

        gm = GridMap(H, W, diag=self.get_parameter('use_diagonal').value, diag_safety=True)

        threshold = self.get_parameter('obstacle_threshold').value

        gm.grid[:, :] = 0
        gm.grid[data >= threshold] = 1

        self.get_logger().info(
            f"GridMap built: H={H}, W={W}, obst_thresh={threshold}, "
            f"diag={gm.diag}, diag_safety={gm.diag_safety}"
        )
        return gm

    # start / goal setting
    def _set_start_goal(self, gm: GridMap):
        H, W = gm.height, gm.width

        sr = self.get_parameter('start_row').value
        sc = self.get_parameter('start_col').value
        gr = self.get_parameter('goal_row').value
        gc = self.get_parameter('goal_col').value

        if sr < 0 or sc < 0 or gr < 0 or gc < 0:
            row = H // 2
            start: Optional[Coord] = None
            goal: Optional[Coord] = None

            for c in range(W):
                if gm.is_free((row, c)):
                    if start is None:
                        start = (row, c)
                    goal = (row, c)  # last free == goal

            if start is None or goal is None or start == goal:
                return

            gm.set_start(start)
            gm.set_goal(goal)

            self.get_logger().info(
                f"Auto start/goal set: start={gm.start}, goal={gm.goal}"
            )
        else:
            if gm.is_free((sr, sc)) and gm.is_free((gr, gc)):
                gm.set_start((sr, sc))
                gm.set_goal((gr, gc))
                self.get_logger().info(
                    f"Start/Goal from param: start={gm.start}, goal={gm.goal}"
                )
            else:
                self.get_logger().warn(
                    f"Param으로 지정한 start/goal 중 장애물이 포함되어 있습니다. "
                    f"start=({sr},{sc}), goal=({gr},{gc})"
                )

    # run algorithms' steps
    def _run_algorithm(self, algo: str, heuristic: str, gm: GridMap) -> Optional[List[Coord]]:
        algo = algo.lower()
        heuristic = heuristic.lower()

        if algo == 'dfs':
            gen = dfs_steps(gm)
        elif algo == 'bfs':
            gen = bfs_steps(gm)
        elif algo == 'dijkstra':
            gen = dijkstra_steps(gm)
        elif algo == 'astar':
            gen = astar_steps(gm, heuristic=heuristic)
        else:
            self.get_logger().error(f"지원하지 않는 algorithm: {algo}")
            return None

        last_snapshot = None
        for snap in gen:
            last_snapshot = snap
            if snap.get("done") and snap.get("path") is not None:
                return snap["path"]

        if last_snapshot and last_snapshot.get("path"):
            return last_snapshot["path"]

        return None

    def _build_path_msg(self, path_cells: List[Coord], map_msg: OccupancyGrid) -> Path:
        path = Path()
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        path.header.frame_id = frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        res = map_msg.info.resolution
        origin = map_msg.info.origin

        for (r, c) in path_cells:
            px = origin.position.x + (c + 0.5) * res
            py = origin.position.y + (r + 0.5) * res

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # yaw=0

            path.poses.append(pose)

        return path


def main(args=None):
    rclpy.init(args=args)
    node = SimpleGridPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
