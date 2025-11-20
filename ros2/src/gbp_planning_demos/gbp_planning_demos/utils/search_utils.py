from __future__ import annotations
from typing import Dict, Tuple, List, Optional
import math

Coord = Tuple[int, int]

# return the path which the algorithm has found.
def reconstruct_path (parent: Dict[Coord, Optional[Coord]], goal: Coord) -> List[Coord]:
    if goal not in parent: return []
    cur = goal
    path = [cur] # goal, goal-1, goal-2, ... , start
    # parent[S] == None
    while parent[cur] is not None:
        cur = parent[cur]
        path.append(cur)
    path.reverse()
    return path

# h(n)
def manhattan(a: Coord, b: Coord) -> float:
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def euclidean(a: Coord, b: Coord) -> float:
    return math.hypot(a[0]-b[0], a[1]-b[1])

def octile(a: Coord, b: Coord) -> float:
    dx, dy = abs(a[0]-b[0]), abs(a[1]-b[1])
    F = 1.0
    D2 = 1.41421356237
    return F*(dx+dy) + (D2 - 2*F)*min(dx, dy)
