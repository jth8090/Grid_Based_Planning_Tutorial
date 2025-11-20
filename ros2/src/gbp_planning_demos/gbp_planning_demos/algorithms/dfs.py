from ..utils.search_utils import reconstruct_path

def dfs_steps(gm):

    if gm.start is None or gm.goal is None:
        yield {
            "visited": set(), 
            "frontier": [], 
            "path": None, 
            "done": False
        }
        return
    # from start here
    sr, sc = gm.start 
    gr, gc = gm.goal
    
    stack = [(sr, sc)]
    visited = {(sr, sc)}
    parent = {(sr, sc): None}

    def snapshot(done=False, path_cells=None):
        return {
            "visited": set(visited), 
            "frontier": list(stack), 
            "path": path_cells, 
            "done": done
        }

    # initialize
    yield snapshot(done=False)

    while stack:
        r, c = stack.pop()                 # pop

        # goal check after pop
        if (r, c) == (gr, gc):
            dfs_path = reconstruct_path(parent, (r, c))
            yield snapshot(done=True, path_cells=dfs_path)
            return

        # LIFO -> reverse NESW
        neighs = list(gm.neighbors(r, c, with_cost=False))
        for nr, nc in reversed(neighs):
            if (nr, nc) not in visited:
                visited.add((nr, nc))
                parent[(nr, nc)] = (r, c)
                stack.append((nr, nc))     # push

        yield snapshot(done=False)

    # failure
    yield snapshot(done=True, path_cells=None)