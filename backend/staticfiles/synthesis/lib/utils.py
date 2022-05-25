import numpy as np
from time import sleep, time

def create_ndbuffer(shape, dtype, buffer):
    return np.ndarray(shape, dtype=dtype, buffer=buffer)


class Synchronise:
    def __init__(self, interval) -> None:
        self.interval = interval
        self.prev_time = None

    def __call__(self):
        if self.prev_time is None:
            self.prev_time = time()
        else:
            sleep(max(0, self.interval - (time() - self.prev_time)))
            self.prev_time = time()

        # print(id(self), 'Executing:', time())
