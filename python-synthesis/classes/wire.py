import multiprocessing
import copy

class Wire:
    def __init__(self, identifier, sh_mem, lock):

        self.id = identifier
        self.value = sh_mem
        #self.addr = hex(id(sh_mem))
        self.addr = hex(id(self.value))
        self.lock = lock

    def read(self):

        self.lock.acquire()
        return_value = self.value.copy()
        self.lock.release()
        return return_value

        '''
            #Faster but might cause synchronization issues.
            while(self.lock.locked()):
                pass
            return self.value
        '''

    def write(self, to_write):

        self.lock.acquire()
        self.value = to_write.copy()
        self.lock.release()
        return
