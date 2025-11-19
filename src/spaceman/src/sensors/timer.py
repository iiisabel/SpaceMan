"""
timer.py
一个可直接调用的极简计时器
"""

import time

class SimpleTimer:
    """
    interval: resolution of time step
    """
    def __init__(self):
        # self.interval = interval
        self.reset()
        self._init_time = self._last_real
    
    @property
    def init_time(self):
        return self._init_time
    
    @property
    def time(self):
        return time.perf_counter() - self._init_time

    def reset(self):
        """把计时器归零"""
        self._step = 0
        self._last_real = time.perf_counter()
    
    
    def step(self):
        self._step += 1
        # self._last_real = time.perf_counter()
        return self._step, self.time