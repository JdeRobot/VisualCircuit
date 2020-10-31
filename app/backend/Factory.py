import numpy as np
import threading
from importlib import import_module
from time import sleep, time
from wires.messenger import send_message
from wires.wire_img import Wire_Read, Wire_Write

ticks = 0

def initialize_block(method_name, input_wires, output_wires, parameters, flags):

    inputs, outputs = create_connections(input_wires, output_wires)
    
    if flags[1] == 1:
        monitoring_thread = threading.Thread(target=frequency_monitor)
        monitoring_thread.start()
    
    try:
        execute()
        
    except KeyboardInterrupt:
    
        monitoring_thread.terminate()
	    
        for memory in outputs:
    	    memory.release()
		    
        exit(0)

def create_connections(input_wires, output_wires):

    inputs = []
    for wire in input_wires:
         inputs.append(Wire_Read(wire))
    
    outputs = []
    for wire in output_wires:
	    outputs.append(Wire_Write(wire))
	    
	return inputs, outputs

def execute(method_name, inputs, outputs, parameters):
    
    module, function = method_name.rsplit('.',1)
    mod = import_module(module)
    method = getattr(mod, function)
    method(inputs, outputs, parameters)


def frequency_monitor(self):

    while(True):
    
        start_time = time()
        sleep(5)
        
        global ticks
        frequency = ticks/(time()-start_time)
        ticks = 0
        to_send = [self.method_name, str(frequency)]
        send_message(to_send)

