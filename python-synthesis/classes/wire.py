import threading

class Wire:
    def __init__(self, identifier):
        self.id = identifier
        self.value = None
        self.lock = threading.Lock()

    def read(self):
        if self.value is not None:
            self.lock.acquire()
            return_value = self.value.copy()
            self.lock.release()
            return return_value
        return None

    def write(self, to_write):
        if to_write is not None:
            self.lock.acquire()
            self.value = to_write.copy()
            self.lock.release()
        return
