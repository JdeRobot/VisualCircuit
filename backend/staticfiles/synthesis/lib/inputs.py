from multiprocessing import shared_memory, Condition

import numpy as np
from lib.exceptions import InvalidInputNameException
from lib.utils import create_ndbuffer


def create_readonly_wire(name):
    try:
        shm = shared_memory.SharedMemory(name, create=False)
    except FileNotFoundError:
        shm = None
    return shm

def create_number_wire(name, size):
        try:
            shm = shared_memory.SharedMemory(name=name)
        except:
            shm = shared_memory.SharedMemory(name=name, create=True, size=size)
        # self.shms.append(shm)
        return shm

class Inputs:

    ENABLE_NAME = "Enable"
    def __init__(self, input_data) -> None:
        self.inputs = input_data
        self._enable_data = self.inputs[Inputs.ENABLE_NAME] if Inputs.ENABLE_NAME in self.inputs else None

    def _init_enabled(self) -> None:
        # This function is redundant for now, its not being used anywhere
        self._enable_data = self.inputs[Inputs.ENABLE_NAME] if Inputs.ENABLE_NAME in self.inputs else None
        self._enable_condition = Condition(self.inputs[Inputs.ENABLE_NAME]["lock"]) if self.enable_wire else None

    def read(self, name):
        if self.inputs.get(name) is None:
            raise InvalidInputNameException(f"{name} is not declared in inputs")

        if self.inputs[name].get("created", False):
            # Do this if read wire has been created
            # Read data from the different buffers of the SHM Objects
            dim = create_ndbuffer((1,), np.int64, self.inputs[name]["dim"].buf)[:][0]
            shape = create_ndbuffer((dim,), np.int64, self.inputs[name]["shape"].buf)
            type = create_ndbuffer((1,), '<U6', self.inputs[name]["type"].buf)[:][0]
            data = create_ndbuffer(shape, type, self.inputs[name]["data"].buf)
        else:
            # Do this if read wire has not been created
            # Create required wires(SHM Objects) to read data from the outputs
            wire_name = self.inputs[name]["wire"]
            data_wire = create_readonly_wire(wire_name)
            dim_wire = create_readonly_wire(wire_name + "_dim")
            shape_wire = create_readonly_wire(wire_name + "_shape")
            type_wire = create_readonly_wire(wire_name + "_type")

            if data_wire is None or shape_wire is None or dim_wire is None or type_wire is None:
                return None

            # Store SHM Object in "dim"
            self.inputs[name]["dim"] = dim_wire
            # Read dimension data from the object's buffer
            dim = create_ndbuffer((1,), np.int64, dim_wire.buf)[:][0]
            # Store SHM Object in "type"
            self.inputs[name]["type"] = type_wire
            # Read type data from the object's buffer
            type = create_ndbuffer((1,), '<U6', type_wire.buf)[:][0]
            # In case type isn't defined return None
            if not type:
                return None 

            # Store SHM Object in "shape"
            self.inputs[name]["shape"] = shape_wire
            # Read shape data from the object's buffer
            shape = create_ndbuffer((dim,), np.int64, shape_wire.buf)

            # Store SHM Object in "data"
            self.inputs[name]["data"] = data_wire
            # Read data from the object's buffer
            data = create_ndbuffer(shape, type[:], data_wire.buf)

            # Mark wire as created
            self.inputs[name]["created"] = True

        return data

    def _read_npy_matrix(self, name, dtype):
        if self.inputs[name].get("created", False):
            dim = create_ndbuffer((1,), np.int64, self.inputs[name]["dim"].buf)[:][0]
            shape = create_ndbuffer((dim,), np.int64, self.inputs[name]["shape"].buf)
            data = create_ndbuffer(shape, dtype, self.inputs[name]["data"].buf)
        else:
            wire_name = self.inputs[name]["wire"]
            data_wire = create_readonly_wire(wire_name)
            shape_wire = create_readonly_wire(wire_name + "_shape")
            dim_wire = create_readonly_wire(wire_name + "_dim")
            if data_wire is None or shape_wire is None or dim_wire is None:
                return None

            self.inputs[name]["dim"] = dim_wire
            dim = create_ndbuffer((1,), np.int64, dim_wire.buf)[:][0]
            self.inputs[name]["shape"] = shape_wire
            shape = create_ndbuffer((dim,), np.int64, shape_wire.buf)
            self.inputs[name]["data"] = data_wire
            data = create_ndbuffer(shape, dtype, data_wire.buf)
            self.inputs[name]["created"] = True

        return data

    def read_image(self, name):
        if self.inputs.get(name) is None:
            raise InvalidInputNameException(f"{name} is not declared in inputs")

        data = self._read_npy_matrix(name, np.uint8)
        return data

    def read_number(self, name):
        if self.inputs.get(name) is None:
            raise InvalidInputNameException(f"{name} is not declared in inputs")

        number = None
        if self.inputs[name].get("created", False):
            number = create_ndbuffer((1,), np.float64, self.inputs[name]["data"].buf)
        else:
            wire_name = self.inputs[name]["wire"]
            data_wire = create_readonly_wire(wire_name)
            if data_wire is None:
                return None
            self.inputs[name]["data"] = data_wire
            number = create_ndbuffer((1,), np.float64, data_wire.buf)
            self.inputs[name]["created"] = True

        return number

    def read_string(self, name):
        if self.inputs.get(name) is None:
            raise InvalidInputNameException(f"{name} is not declared in inputs")

        string = None
        if self.inputs[name].get("created", False):
            string = create_ndbuffer((1,), '<U64', self.inputs[name]["data"].buf)
        else:
            wire_name = self.inputs[name]["wire"]
            data_wire = create_readonly_wire(wire_name)
            if data_wire is None:
                return None
            self.inputs[name]["data"] = data_wire
            string = create_ndbuffer((1,), '<U64', data_wire.buf)
            self.inputs[name]["created"] = True
        
        return string

    def read_array(self, name):
        if self.inputs.get(name) is None:
            raise InvalidInputNameException(f"{name} is not declared in inputs")

        data = self._read_npy_matrix(name, np.float64)
        return data


    @property
    def enabled(self) -> bool:
        # No enable wire used, so enabled by default
        if self._enable_data is None:
            return True

        _enabled = self.read_number(Inputs.ENABLE_NAME)

        # If enable wire is present but, not initialized, return False
        if _enabled is None:
            return False

        return np.isclose(_enabled, np.array([1.0]))



    @enabled.setter
    def enabled(self, _enabled: bool):
        # If no wire exists, we cannot set anything, it is true by default
        # TODO: Ideally one should be able to trigger a block on and off even without an enable wire
        # Can we force there to be an enable slot in all blocks? Or is another approach a better idea? 
        if self._enable_data is None:
            return
            
        self._enable_data["lock"].acquire()
        if self._enable_data.get("created", False):
            a = np.array([1])
            if _enabled:
                # print("Triggering wire online")
                a = np.array([1])
            else:
                # print("Disabling wire")
                a = np.array([0])

            b = np.ndarray(a.shape, dtype=a.dtype, buffer=self._enable_data["data"].buf)
            b[:] = a[:]


        else:
            # Wire doesn't exist yet, ideally has to be created and set
            # TODO: Is this case necessary?
            # Yep case is definitely necessary especially by this implementation
            
            # If the command has been give to enable the wire 
            if _enabled:
                
                wire_name = self._enable_data["wire"]
                # Value of the wire to be set
                wire_val = np.array([1])
                # Create a new shared memory object in the "data" key of the _enable_data dictionary
                self._enable_data["data"] = create_number_wire(wire_name, a.nbytes)
                data_wire = np.ndarray(wire_val.shape, dtype=np.float64, buffer=self._enable_data["data"].buf)
                data_wire[:] = wire_val[:]
                # Mark wire as created, since it has been created
                self._enable_data["created"] = True

        self._enable_data["lock"].release()
        