from importlib import import_module
from wires.wire_img import Wire_Read, Wire_Write


def initialize_block(method_name, input_wires, output_wires, parameters, flags):

    inputs, outputs = create_connections(input_wires, output_wires)
    
    try:
        execute(method_name, inputs, outputs, parameters)
        
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



