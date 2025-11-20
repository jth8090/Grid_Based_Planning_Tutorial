import heapq
from utils.search_utils import reconstruct_path

def dijkstra_steps(gm):

    if gm.start is None or gm.goal is None:
        yield {
            "visited": set(), 
            "frontier": [], 
            "path": None, 
            "done": False
        }
        return

    sr, sc = gm.start
    gr, gc = gm.goal
    
    H, W = gm.height, gm.width

    g_cost = [[float('inf')]*W for _ in range(H)]
    g_cost[sr][sc] = 0.0
    parent = {(sr, sc): None}
    priority_queue = [(0.0, sr, sc)]
    visited = set()

    def snapshot(done=False, path_cells=None):
        frontier_cells = [(r, c) for _, r, c in priority_queue]
        return {
            "visited": set(visited), 
            "frontier": frontier_cells, 
            "path": path_cells, 
            "done": done
        }

    yield snapshot(False)

    while priority_queue:
        # pop first
        g, r, c = heapq.heappop(priority_queue)
        
        if (r, c) in visited:
            continue
        visited.add((r, c))

        # goal check after popping
        if (r, c) == (gr, gc):
            dijk_path = reconstruct_path(parent, (r, c))
            yield snapshot(True, path_cells=dijk_path)
            return

        for nr, nc, move_cost in gm.neighbors(r, c, with_cost=True):
            new_g = g + move_cost
            # edge relaxation
            if new_g < g_cost[nr][nc]:
                g_cost[nr][nc] = new_g
                parent[(nr, nc)] = (r, c)
                heapq.heappush(priority_queue, (new_g, nr, nc))

        yield snapshot(False)

    yield snapshot(True, None)