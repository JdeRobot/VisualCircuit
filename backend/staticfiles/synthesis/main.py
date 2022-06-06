import importlib
import json
import multiprocessing
import random
import string
from multiprocessing import shared_memory
from time import sleep

from lib.inputs import Inputs
from lib.outputs import Outputs
from lib.parameters import Parameters
from lib.utils import Synchronise


BLOCK_DIRECTORY = 'modules'
FUNCTION_NAME = 'main'


def clean_shared_memory(names):
    all_names = names[:]
    all_names.extend([name + "_dim" for name in names])
    all_names.extend([name + "_shape" for name in names])
    for name in all_names:
        try:
            shm = shared_memory.SharedMemory(name, create=False)
            shm.close()
            shm.unlink()
        except FileNotFoundError:
            pass


def main():
    """
    Main function
    """
    with open("data.json") as json_file:
        data = json.load(json_file)

    blocks = data["blocks"]
    wires = data["wires"]
    parameters = data["parameters"]
    synchronize_frequency = data["synchronize_frequency"]

    block_data = {}
    wire_names = []

    for wire in wires:
        source = wire["source"]
        target = wire["target"]

        if source["block"] in parameters:
            block_data[target["block"]] = block_data.get(
                target["block"], {"inputs": {}, "outputs": {}, "parameters": {}}
            )
            for param in parameters[source["block"]]:
                parameter_data = {param["name"]: param["value"]}
                block_data[target["block"]]["parameters"].update(parameter_data)
        else:
            wire_name = "".join(
                random.choices(string.ascii_uppercase + string.digits, k=10)
            )
            wire_names.append(wire_name)
            output_data = {source["name"]: {"wire": wire_name}}
            input_data = {target["name"]: {"wire": wire_name}}
            block_data[source["block"]] = block_data.get(
                source["block"], {"inputs": {}, "outputs": {}, "parameters": {}}
            )
            block_data[source["block"]]["outputs"].update(output_data)
            block_data[target["block"]] = block_data.get(
                target["block"], {"inputs": {}, "outputs": {}, "parameters": {}}
            )
            block_data[target["block"]]["inputs"].update(input_data)


    for block in blocks:
        if blocks[block]["type"] in parameters:
            block_data[block] = block_data.get(block, {"inputs": {}, "outputs": {}, "parameters": {}})
            for param in parameters[ blocks[block]["type"]]:
                parameter_data = {param["name"]: param["value"]}
                block_data[block]["parameters"].update(parameter_data)
        if block in synchronize_frequency or blocks[block]["type"] in synchronize_frequency:
            block_data[block] = block_data.get(block, {"inputs": {}, "outputs": {}, "parameters": {}})
            block_data[block]["frequency"] = synchronize_frequency.get(block, synchronize_frequency.get(blocks[block]["type"], 30))

    processes = []

    for block_id, block in blocks.items():
        name = BLOCK_DIRECTORY + "." + block["name"]
        mod = importlib.import_module(name)
        method = method = getattr(mod, FUNCTION_NAME)

        inputs = Inputs(block_data[block_id]["inputs"])
        outputs = Outputs(block_data[block_id]["outputs"])
        parameters = Parameters(block_data[block_id]["parameters"])
        freq = block_data[block_id]["frequency"]
        processes.append(
            multiprocessing.Process(target=method, args=(inputs, outputs, parameters, Synchronise(1 / (freq if freq != 0 else 30))))
        )

    for process in processes:
        process.start()

    try:
        while True:
            sleep(10)
    except KeyboardInterrupt:
        for process in processes:
            process.terminate()
            process.join()
        clean_shared_memory(wire_names)


if __name__ == "__main__":
    main()
