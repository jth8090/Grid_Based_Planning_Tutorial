import math
import numpy as np

# NESW (N → E → S → W)
DIR4 = [(-1, 0), (0, 1), (1, 0), (0, -1)]
# 8-connectivity (N → NE → E → SE → S → SW → W → NW)
DIR8 = [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]

class GridMap:
    def __init__(self, H, W, diag: bool = False, diag_safety: bool = True):
        self.height = int(H)
        self.width  = int(W)
        self.grid   = np.zeros((self.height, self.width), dtype=int)  # 0=free, 1=obstacle
        self.start  = None  # (r, c)
        self.goal   = None  # (r, c)
        self.diag = bool(diag)                # 4 or 8 -connectivity
        self.diag_safety = bool(diag_safety)  # anti-corner cut
        self.costmap = None        
        self.cost_add_weight = 1.0 


    # utility
    def is_free(self, rc):
        r, c = rc
        return (0 <= r < self.height) and (0 <= c < self.width) and (self.grid[r, c] == 0)

    def set_start(self, rc):
        if self.is_free(rc):
            self.start = tuple(rc)

    def set_goal(self, rc):
        if self.is_free(rc):
            self.goal = tuple(rc)

    def set_connectivity(self, diag: bool):
        self.diag = bool(diag)

    def set_diag_safety(self, on: bool):
        self.diag_safety = bool(on)
        
    def set_costmap(self, cm):
        self.costmap = cm

    def set_cost_weight(self, w: float):
        self.cost_add_weight = float(max(0.0, w))

    # neighbors func. (outgoing)
    def neighbors(self, r: int, c: int, with_cost: bool = False):

        dirs = DIR8 if self.diag else DIR4
        
        for dr, dc in dirs:
            nr, nc = r + dr, c + dc
            
            # boundary check
            if not (0 <= nr < self.height and 0 <= nc < self.width):
                continue
            # obstacle check
            if not self.is_free((nr, nc)):
                continue

            # anti corner cut -> adj cell chekcing
            if self.diag and self.diag_safety and dr != 0 and dc != 0:
                if not (self.is_free((r, nc)) and self.is_free((nr, c))):
                    continue

            # this is for Dijkstra & A*
            if with_cost:     
                # move_cost
                move_cost = 1.0 if (dr == 0 or dc == 0) else math.sqrt(2.0)
                
                # additional cost (from costmap)
                add = 0.0
                cm = self.costmap
                if cm is not None:
                    try:
                        v = float(cm.value_at(nr, nc))
                    except AttributeError:
                        v = float(cm.cost[nr, nc])
                    if v < 0.0:
                        v = 0.0
                    add = self.cost_add_weight * v
                    
                # neighbors' coordinates + edge cost    
                yield (nr, nc, move_cost + add)
                
            else:
                yield (nr, nc)
