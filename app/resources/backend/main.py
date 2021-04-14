import os
import tkinter
import numpy as np
import multiprocessing
from lib.block import Block
from utils.wires.wire_str import read_string

def loginfo(to_display):
    text.insert(tkinter.END, to_display+'\n')
    text.pack()

#------------------- Reads path from provided arguments and loads files --------------------------#
def initialize():

    import argparse
    parser = argparse.ArgumentParser(description='Python Synthesizer')
    parser.add_argument('--path', help='Enter path to mapping.vc', default="main_test/mapping.vc")
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

#----- Generates Python Scripts -----#
def generate_py_script(working_dir, script, script_name):

    os.chdir(working_dir)
    generator = open('modules/'+script_name+".py", "w")
    generator.write(script)
    generator.close()
#------------------------------------#

#----- Program exit Logic ------#
def end_progam():

    root.destroy()

    for process in processes:
        process.terminate()
        process.join()

    exit(0)
#-------------------------------#

#----- Frequency Monitor ------#

def monitor_frequency(memories):

    from time import sleep

    no_of_lines = len(memories)
    for lines in range(no_of_lines):
    	text.delete("1.0", tkinter.END)
    
    loginfo("<---------- Frequency Monitor ---------->\n\n")
    for memory in memories:
        message = memory.get()
        
        for msg in message:
            loginfo(msg)
        loginfo("----------------------------------------")
        
    sleep(0.2)

#-------------------------------#


#--------------------------- Creates Python Application from .vc File -------------------------#
def build(working_dir):

    # Reading Front-End File
    data = initialize()

    # Creating dictionary of block types and their names
    # Example: (d62a6403-2db3-4832-9f4f-0f2dc8ac407d, Camera)
    info = data['dependencies']
    keys = info.keys()
    type_names = []

    for key in keys:
    
        name = info[key]['package']['name']
        name += info[key]['package']['version'].replace('.', '')
        type_names.append((key, name))
        loginfo("Creating Block: "+ name)
        

    # Creating dictionary of block types and their ID's
    identifiers = data['design']['graph']['blocks']

    blocks = []
    parameters_list = []
    count = 1
    for block in identifiers:

        hex_id, block_type = block['id'], block['type']

        if block_type == 'basic.code':
            
            code_name = "Code_"+str(count)
            count += 1

            script = block['data']['code']
            generate_py_script(working_dir, script, code_name)

            new_block = Block(hex_id, block_type)
            new_block.name = code_name
            blocks.append(new_block)           

        elif block_type == 'basic.constant':
            parameters_list.append((hex_id, block['data']['value']))

        elif block_type == 'basic.info':
            pass

        else:
            new_block = Block(hex_id, block_type)
            new_block.set_name(type_names)
            blocks.append(new_block)


    ############# Generating Scripts for User Code & Setting Parameters #################
    for key in keys:
        
        components = info[key]['design']['graph']['blocks']
            
        for element in components:
                
            if element['type'] == 'basic.code':

                script = element['data']['code']
                scr_name = info[key]['package']['name']
                scr_name += info[key]['package']['version'].replace('.', '')
                generate_py_script(working_dir, script, scr_name)
            
            elif element['type'] == 'basic.constant':
                
                for block in blocks:
                    if block.id_type == key:
                        block.add_parameter(element['data']['value'], element['id'])
    #####################################################################################

    # Reading Wires Mapping
    wires = data['design']['graph']['wires']


    ###################### Setting up communication between blocks ######################
    loginfo("Setting up connections...")

    
    for wire in wires:

        src_block, src_port = wire['source']['block'], wire['source']['port']
        tgt_block, tgt_port = wire['target']['block'], wire['target']['port']
        
        wire_id = src_block+str(src_port)


        # Connecting Paramters to Blocks
        if src_port == 'constant-out':

            param_value = None
            for param in parameters_list:
                if param[0] == src_block:
                    param_value = param[1]
                    break

            for block in blocks:
                if tgt_block == block.id:
                    block.add_parameter(param_value, tgt_port)
                    break

        # Connecting Wires to Blocks
        else:

            for block in blocks:
                if tgt_block == block.id:
                    block.connect_input_wire(wire_id, tgt_port)
                    break

            for block in blocks:
                if src_block == block.id:
                    block.connect_output_wire(wire_id, src_port)
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

    root_dir = os.getcwd() 
    # Configuring path for working directory
    from pathlib import Path
    working_dir = str(Path.home()) + '/vc_workspace'
    Path(working_dir).mkdir(parents=True, exist_ok=True)
    Path(working_dir+'/modules').mkdir(parents=True, exist_ok=True)
    
    loginfo("Building Application...")
    blocks = build(working_dir)

    # Creating processes and assigning blocks. 
    loginfo("Creating Processes...")
    processes = []
    
    os.chdir(working_dir)
    mkpkg = open("modules/__init__.py", "w")
    mkpkg.close()

    os.chdir(root_dir)
    
    from importlib import import_module
    import sys
    sys.path.append(working_dir)
    for i, element in enumerate(blocks):

        block_name = element.name

        method_name = 'modules.' + block_name + '.loop'
        module, function = method_name.rsplit('.',1)

        mod = import_module(module)
        method = getattr(mod, function)

        element.sort_ports()
        input_args = element.input_ports
        output_args = element.output_ports
        parameters = element.parameters

        flags = [1,0,0]
        processes.append(multiprocessing.Process(target=method,
        args=(block_name, input_args, output_args, parameters, flags,)))

        
    loginfo("Starting Application")
    
    root.update_idletasks()
    root.update()
    
    for process in processes:
        process.start()

    memories = []
    for process in processes:
        memories.append(read_string(str(process.pid)))
    
    while True:
    
    	monitor_frequency(memories)
    	root.update_idletasks()
    	root.update()
