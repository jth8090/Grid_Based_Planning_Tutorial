import heapq, math
from utils.search_utils import reconstruct_path

HEURISTICS = {
    "manhattan": lambda r,c,gr,gc: abs(r-gr) + abs(c-gc),
    "octile":    lambda r,c,gr,gc: (max(abs(r-gr),abs(c-gc)) + (math.sqrt(2)-1.0)*min(abs(r-gr),abs(c-gc))),
    "euclidean": lambda r,c,gr,gc: math.hypot(r-gr, c-gc),
}

def astar_steps(gm, heuristic="manhattan"):

    if gm.start is None or gm.goal is None:
        yield {
            "visited": set(), 
            "frontier": [], 
            "path": None, 
            "done": False}
        return

    hfun = HEURISTICS.get(heuristic, HEURISTICS["manhattan"])
    sr, sc = gm.start
    gr, gc = gm.goal

    g_score = {(sr, sc): 0.0}
    parent  = {(sr, sc): None}
    priority_queue = [(hfun(sr, sc, gr, gc), 0.0, sr, sc)]  # (f, g, r, c)
    visited = set()

    def snapshot(done=False, path_cells=None):
        frontier_cells = [(r, c) for _, __, r, c in priority_queue]
        return {
            "visited": set(visited), 
            "frontier": frontier_cells,
            "path": path_cells, 
            "done": done
        }

    yield snapshot(False)

    while priority_queue:
        # f_now == f >> not used in while (only used for key in heapq)
        f_now, g_now, r, c = heapq.heappop(priority_queue)
        if (r, c) in visited:
            continue
        visited.add((r, c)) # CLOSED

        # stop-on-pop >> optimality
        if (r, c) == (gr, gc):
            Astar_path = reconstruct_path(parent, (r, c))
            yield snapshot(True, path_cells=Astar_path)
            return

        for nr, nc, move_cost in gm.neighbors(r, c, with_cost=True):
            new_g = g_now + move_cost
            if new_g < g_score.get((nr, nc), float('inf')):
                g_score[(nr, nc)] = new_g
                parent[(nr, nc)] = (r, c)
                f_next = new_g + hfun(nr, nc, gr, gc)
                heapq.heappush(priority_queue, (f_next, new_g, nr, nc))

        yield snapshot(False)

    yield snapshot(True, None)
