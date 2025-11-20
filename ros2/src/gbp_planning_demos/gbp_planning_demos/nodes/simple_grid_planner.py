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
    """
    /map(OccupancyGrid)을 받아서 GridMap으로 변환 후
    선택한 알고리즘(DFS/BFS/Dijkstra/A*)으로 경로를 계산하고
    nav_msgs/Path로 퍼블리시하는 데모 노드.
    """

    def __init__(self):
        super().__init__('simple_grid_planner')

        # --- 파라미터 선언 ---
        self.declare_parameter('algorithm', 'bfs')          # bfs / dfs / dijkstra / astar
        self.declare_parameter('heuristic', 'manhattan')    # astar일 때 manhattan / octile / euclidean
        self.declare_parameter('use_diagonal', True)       # 4연결 vs 8연결
        self.declare_parameter('obstacle_threshold', 50)    # OccGrid 값 >= threshold → 장애물
        self.declare_parameter('start_row', 10)             # <0 이면 자동 선택
        self.declare_parameter('start_col', 10)
        self.declare_parameter('goal_row', 190)
        self.declare_parameter('goal_col', 190)
        self.declare_parameter('frame_id', 'map')

        # --- QoS: /map 은 Transient Local 이라 맞춰주는 게 좋음 ---
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

    # ------------------------------------------------------------------
    # /map 콜백: 한 번만 받아서 바로 경로 계산 후 퍼블리시
    # ------------------------------------------------------------------
    def map_callback(self, msg: OccupancyGrid):
        if self._map_received:
            return
        self._map_received = True

        self.get_logger().info("Received /map. Building GridMap and planning ...")

        gm = self._build_gridmap_from_occupancy(msg)

        # start / goal 자동 설정 또는 파라미터 기반 설정
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

    # ------------------------------------------------------------------
    # OccupancyGrid → GridMap
    # ------------------------------------------------------------------
    def _build_gridmap_from_occupancy(self, msg: OccupancyGrid) -> GridMap:
        H = msg.info.height
        W = msg.info.width
        data = np.array(msg.data, dtype=np.int16).reshape(H, W)

        gm = GridMap(H, W, diag=self.get_parameter('use_diagonal').value, diag_safety=True)

        threshold = self.get_parameter('obstacle_threshold').value

        # -1: unknown → 일단 free 취급
        # >= threshold: obstacle
        gm.grid[:, :] = 0
        gm.grid[data >= threshold] = 1

        self.get_logger().info(
            f"GridMap built: H={H}, W={W}, obst_thresh={threshold}, "
            f"diag={gm.diag}, diag_safety={gm.diag_safety}"
        )
        return gm

    # ------------------------------------------------------------------
    # Start / Goal 설정 (파라미터 없으면 가운데 row에서 좌→우 스캔하며 자동 선택)
    # ------------------------------------------------------------------
    def _set_start_goal(self, gm: GridMap):
        H, W = gm.height, gm.width

        sr = self.get_parameter('start_row').value
        sc = self.get_parameter('start_col').value
        gr = self.get_parameter('goal_row').value
        gc = self.get_parameter('goal_col').value

        # 자동 설정: 가운데 row에서 가장 왼쪽 free → start, 가장 오른쪽 free → goal
        if sr < 0 or sc < 0 or gr < 0 or gc < 0:
            row = H // 2
            start: Optional[Coord] = None
            goal: Optional[Coord] = None

            for c in range(W):
                if gm.is_free((row, c)):
                    if start is None:
                        start = (row, c)
                    goal = (row, c)  # 마지막 free를 goal로

            if start is None or goal is None or start == goal:
                self.get_logger().warn(
                    "자동 start/goal 설정 실패 또는 같음. "
                    "직접 start_row/col, goal_row/col 파라미터를 설정하는 것을 권장합니다."
                )
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

    # ------------------------------------------------------------------
    # 알고리즘 실행: dfs / bfs / dijkstra / astar
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # Grid cell 리스트 → nav_msgs/Path 변환
    # ------------------------------------------------------------------
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
