#Libraries
import multiprocessing
import numpy as np
import ctypes
from classes.wire import Wire

#Functions
def load_JSON(file_path):

    import json
    with open(file_path) as obj:
        data = json.load(obj)
    return data

def initialize():

    import argparse
    parser = argparse.ArgumentParser(description='Python Synthesizer')
    parser.add_argument('--wires', help='Enter path to wires.json', default="example/wires.json")
    parser.add_argument('--mapping', help='Enter path to mapping.json', default="example/mapping.json")
    args = parser.parse_args()

    wires = load_JSON(args.wires)
    data = load_JSON(args.mapping)

    return wires, data

def shared_array(shape):
    shared_array_base = multiprocessing.Array(ctypes.c_uint8, shape[0]*shape[1])
    shared_array = np.ctypeslib.as_array(shared_array_base.get_obj())
    shared_array = shared_array.reshape(*shape)
    return shared_array

if __name__ == "__main__":

    wires, data = initialize()

    wires_list = []
    for wire in wires:
        wires_list.append( Wire(wire, shared_array((480, 640)), multiprocessing.Lock()) )

    processes = []
    for i, element in enumerate(data["mapping"]):

        from importlib import import_module
        method_name = 'modules.' + element["block_name"] + '.' + element["block_name"]
        module, function = method_name.rsplit('.',1)
        mod = import_module(module)
        method = getattr(mod, function)

        input_args = element["input_wires"]
        output_args = element["output_wires"]
        parameters = element["parameters"]

        processes.append(multiprocessing.Process(target=method,
        args=(wires_list, input_args, output_args, parameters,)))

        processes[i].start()

    for process in processes:
        process.join()
