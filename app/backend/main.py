import ctypes
import tkinter
import numpy as np
import multiprocessing
from block import Block


# Reads path from provided arguments and loads files
def initialize():

    import argparse
    parser = argparse.ArgumentParser(description='Python Synthesizer')
    parser.add_argument('--path', help='Enter path to mapping.vz', default="backend/examples/mapping.vz")
    args = parser.parse_args()

    data = load_JSON(args.path)

    return data

# Loads JSON object created by NodeJS frontend
def load_JSON(file_path):

    import json
    with open(file_path) as obj:
        data = json.load(obj)
    return data

# Program exit
def end_progam():
    quit_program.destroy()

    for process in processes:
        process.terminate()
        process.join()

    exit(0)


if __name__ == "__main__":


    quit_program = tkinter.Tk()
    button = tkinter.Button(quit_program, text = "Your Application is Running!\n Press this Button to Exit!", command = end_progam)
    button.pack()

    # Reading Front-End File
    data = initialize()

    # Creating dictionary of block types and their names
    info = data['dependencies']
    keys = info.keys()
    type_names = []

    for key in keys:
        type_names.append((key, info[key]['package']['name']))

    # Creating dictionary of block types and their ID's
    identifiers = data['design']['graph']['blocks']

    blocks = []

    for block in identifiers:

        new_block = Block(block['id'], block['type'])
        new_block.set_name(type_names)
        blocks.append(new_block)


    # Reading Wires Mapping
    wires = data['design']['graph']['wires']

    dictionary = {}
    for i, wire in enumerate(wires):
        if wire['source']['block'] not in dictionary:
            dictionary[wire['source']['block']] = str(i)


    # Setting up communication between blocks
    for wire in wires:

        wire_id = dictionary[wire['source']['block']]+wire['source']['port']

        input_terminal = wire['target']
        for block in blocks:
            if input_terminal['block'] == block.id:
                block.connect_input_wire(wire_id)
                break

        output_terminal = wire['source']
        for block in blocks:
            if output_terminal['block'] == block.id:
                block.connect_output_wire(wire_id)
                break


    # Creating processes and assigning blocks. 
    processes = []
    for i, element in enumerate(blocks):

        from importlib import import_module
        block_name = element.name
        method_name = 'modules.' + block_name + '.' + block_name
        module, function = method_name.rsplit('.',1)
        mod = import_module(module)
        method = getattr(mod, function)

        element.sort_ports()
        input_args = element.input_ports
        output_args = element.output_ports
        parameters = element.parameters

        processes.append(multiprocessing.Process(target=method,
        args=(input_args, output_args, parameters,)))

        processes[i].start()

    quit_program.mainloop()




