import numpy as np

class CostMap:
    __slots__ = ("cost", "max_value")

    def __init__(self, cost: np.ndarray, max_value: float = 5.0):
        self.cost = np.asarray(cost, dtype=float)
        self.max_value = float(max_value)

    @staticmethod
    def zeros(H, W, max_value: float = 5.0):
        return CostMap(np.zeros((H, W), dtype=float), max_value=max_value)

    def copy(self):
        return CostMap(self.cost.copy(), max_value=self.max_value)

    def clear(self):
        self.cost[:] = 0.0

    def value_at(self, r, c):
        return float(self.cost[r, c])

    def set_value(self, r, c, v: float):
        self.cost[r, c] = max(0.0, float(v))