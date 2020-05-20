#Libraries
import json
import threading
from importlib import import_module
from classes.wire import Wire
from modules.edge_detector import edge_detector
from modules.camera import camera
from modules.screen import screen

#Configurations
WIRES_PATH = "example/wires.json"
MAPPING_PATH = "example/mapping.json"

#Functions
def load_JSON(file_path):

    with open(file_path) as obj:
        data = json.load(obj)
    return data

#cd Desktop/VisualCircuit-V1
#Driver Code
wires = load_JSON(WIRES_PATH)

wires_list = []
for wire in wires:
    wires_list.append(Wire(wire))

data = load_JSON(MAPPING_PATH)

thread_pool = []

for i, element in enumerate(data["mapping"]):
    method_name = element["block_name"]
    possibles = globals().copy()
    possibles.update(locals())
    method = possibles.get(method_name)
    input_args = element["input_wires"]
    output_args = element["output_wires"]
    parameters = element["parameters"]
    print(method_name, input_args, output_args, parameters)
    thread_pool.append(threading.Thread(target=method,
    args=(wires_list, input_args, output_args, parameters,)))
    thread_pool[i].start()
