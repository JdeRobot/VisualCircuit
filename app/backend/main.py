import ctypes
import tkinter
import numpy as np
import multiprocessing
from block import Block

def loginfo(to_display):
    text.insert(tkinter.END, to_display+'\n')

#------------------- Reads path from provided arguments and loads files --------------------------#
def initialize():

    import argparse
    parser = argparse.ArgumentParser(description='Python Synthesizer')
    parser.add_argument('--path', help='Enter path to mapping.vz', default="examples/sample.ice")
    args = parser.parse_args()
    
    loginfo("Creating Project at: "+ args.path)
    data = load_JSON(args.path)

    return data
#-------------------------------------------------------------------------------------------------#

#---- Loads .vc JSON object ----#
def load_JSON(file_path):

    import json
    with open(file_path) as obj:
        data = json.load(obj)
    return data
#-------------------------------#

#----- Program exit Logic ------#
def end_progam():
    root.destroy()

    for process in processes:
        process.terminate()
        process.join()

    exit(0)
#-------------------------------#


#--------------------------- Creates Python Application from .vc File -------------------------#
def build():

    # Reading Front-End File
    data = initialize()

    # Creating dictionary of block types and their names
    # Example: (d62a6403-2db3-4832-9f4f-0f2dc8ac407d, Camera)
    info = data['dependencies']
    keys = info.keys()
    type_names = []

    for key in keys:
    
        name = info[key]['package']['name']
        type_names.append((key, name))
        loginfo("Creating Block: "+ name)
        

    # Creating dictionary of block types and their ID's
    identifiers = data['design']['graph']['blocks']

    blocks = []

    for block in identifiers:

        new_block = Block(block['id'], block['type'])
        new_block.set_name(type_names)
        blocks.append(new_block)


    ########### Generating Scripts for User Code & Setting Parameters ##################
    for key in keys:
        
        components = info[key]['design']['graph']['blocks']
            
        for element in components:
                
            if element['type'] == 'basic.code':
                script = element['data']['code']
                f = open("backend/modules/"+info[key]['package']['name']+".py", "w")
                f.write(script)
                f.close()
            
            elif element['type'] == 'basic.constant':
                
                for block in blocks:
                    if block.id_type == key:
                        block.add_parameter(element['data']['value'])
    #####################################################################################

    # Reading Wires Mapping
    wires = data['design']['graph']['wires']

    dictionary = {}
    for i, wire in enumerate(wires):
        if wire['source']['block'] not in dictionary:
            dictionary[wire['source']['block']] = str(i)


    ###################### Setting up communication between blocks ######################
    loginfo("Setting up connections...")
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
    ######################################################################################
    
    return blocks
    
#-----------------------------------------------------------------------------------------------------#

#Driver Code
if __name__ == "__main__":


    # Creating Exit Button
    root = tkinter.Tk()
    text = tkinter.Text(root)
    button = tkinter.Button(root, text = "Click here to Exit!", command = end_progam)
    text.pack()
    button.pack()

    loginfo("Building Application...")
    blocks = build()

    # Creating processes and assigning blocks. 
    loginfo("Creating Processes...")
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

    loginfo("Starting Application")
    root.mainloop()
        
    
