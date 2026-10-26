"""Pure-Python basket model: counts picked apples up to a fixed capacity (no rclpy)."""


class Basket:

    def __init__(self, capacity=10):
        if capacity <= 0:
            raise ValueError('capacity must be positive')
        self.capacity = capacity
        self.count = 0
        self.total_picked = 0     # cumulative over the whole run
        self.total_unloaded = 0   # apples dropped at the depot

    @property
    def full(self):
        return self.count >= self.capacity

    def add(self):
        """Add one apple. Return False (and change nothing) if the basket is full."""
        if self.full:
            return False
        self.count += 1
        self.total_picked += 1
        return True

    def unload(self):
        """Empty the basket and return the number of apples unloaded."""
        n = self.count
        self.total_unloaded += n
        self.count = 0
        return n
