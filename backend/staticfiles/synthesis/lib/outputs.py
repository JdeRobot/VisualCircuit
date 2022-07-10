from multiprocessing import shared_memory

import numpy as np
from lib.exceptions import InvalidOutputNameException
from lib.utils import create_ndbuffer


class Outputs:
    def __init__(self, output_data) -> None:
        self.outputs = output_data
        self.shms = []

    def _create_wire(self, name, size):
        shm = shared_memory.SharedMemory(name=name, create=True, size=size)
        self.shms.append(shm)
        return shm

    def _share_npy_matrix(self, name, matrix, shape):
        dim = np.array([len(matrix.shape)], dtype=np.int64)
        if self.outputs[name].get("created", False):
            self.outputs[name]["shape"][:] = shape[:]
            self.outputs[name]["data"][:] = matrix[:]
        else:
            wire_name = self.outputs[name]["wire"]
            data_wire = self._create_wire(wire_name, matrix.nbytes)
            shape_wire = self._create_wire(wire_name + "_shape", shape.nbytes)
            dim_wire = self._create_wire(wire_name + "_dim", dim.nbytes)
            self.outputs[name]["dim"] = create_ndbuffer((1,), np.int64, dim_wire.buf)
            self.outputs[name]["dim"][:] = dim[:]
            self.outputs[name]["shape"] = create_ndbuffer(
                shape.shape, shape.dtype, shape_wire.buf
            )
            self.outputs[name]["shape"][:] = shape[:]
            self.outputs[name]["data"] = create_ndbuffer(
                shape, matrix.dtype, data_wire.buf
            )
            self.outputs[name]["data"][:] = matrix[:]
            self.outputs[name]["created"] = True

    def share_image(self, name, image):
        if self.outputs.get(name) is None:
            raise InvalidOutputNameException(f"{name} is not declared in outputs")

        image = np.array(image, dtype=np.uint8)
        if len(image.shape) != 2 and len(image.shape) != 3:
            raise ValueError("Image must be 2D or 3D")

        shape = (
            image.shape
            if len(image.shape) == 3
            else (image.shape[0], image.shape[1], 1)
        )
        shape = np.array(shape, dtype=np.int64)
        self._share_npy_matrix(name, image, shape)

    def share_number(self, name, number):
        if self.outputs.get(name) is None:
            raise InvalidOutputNameException(f"{name} is not declared in outputs")

        if self.outputs[name].get("created", False):
            self.outputs[name]["data"][:] = number
        else:
            wire_name = self.outputs[name]["wire"]
            data_wire = self._create_wire(
                wire_name, np.array([1], dtype=np.float64).nbytes
            )
            self.outputs[name]["data"] = create_ndbuffer(
                (1,), np.float64, data_wire.buf
            )
            self.outputs[name]["data"][:] = number
            self.outputs[name]["created"] = True

    def share_string(self, name, string):
        if self.outputs.get(name) is None:
            raise InvalidOutputNameException(f"{name} is not declared in outputs")

        if self.outputs[name].get("created", False):
            self.outputs[name]["data"][:] = string
        else:
            wire_name = self.outputs[name]["wire"]
            data_wire = self._create_wire(
                wire_name, np.array(string, dtype='<U64').nbytes
            )
            self.outputs[name]["data"] = create_ndbuffer(
                (1,), '<U64', data_wire.buf
            )
            self.outputs[name]["data"][:] = string
            self.outputs[name]["created"] = True

    def share_array(self, name, array):
        if self.outputs.get(name) is None:
            raise InvalidOutputNameException(f"{name} is not declared in outputs")
        array = np.array(array, dtype=np.float64)
        self._share_npy_matrix(name, array, np.array(array.shape, dtype=np.int64))
