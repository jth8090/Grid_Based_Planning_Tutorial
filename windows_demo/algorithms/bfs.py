from collections import deque   # double-ended queue
from utils.search_utils import reconstruct_path

def bfs_steps(gm):

    if gm.start is None or gm.goal is None:
        yield {
            "visited": set(), 
            "frontier": [], 
            "path": None, 
            "done": False
        }
        return

    # when start == goal
    if gm.start == gm.goal:
        path_cells = [gm.start]
        yield {
            "visited": {gm.start}, 
            "frontier": [], 
            "path": path_cells, 
            "done": True
        }
        return

    sr, sc = gm.start
    gr, gc = gm.goal
    
    queue = deque([(sr, sc)])
    visited = {(sr, sc)}
    parent = {(sr, sc): None}

    def snapshot(done=False, path_cells=None):
        return {
            "visited": set(visited), 
            "frontier": list(queue), 
            "path": path_cells, 
            "done": done
        }

    # initialize
    yield snapshot(done=False)

    while queue:
        r, c = queue.popleft()      # dequeue

        # neighbors check
        for nr, nc in gm.neighbors(r, c, with_cost=False):
            if (nr, nc) in visited:
                continue
            visited.add((nr, nc))
            parent[(nr, nc)] = (r, c)
            queue.append((nr, nc))  # enqueue

            if (nr, nc) == (gr, gc):  # check goal after enqueue
                bfs_path = reconstruct_path(parent, (nr, nc))
                yield snapshot(done=True, path_cells=bfs_path)
                return

        yield snapshot(done=False)

    # failure
    yield snapshot(done=True, path_cells=None)
