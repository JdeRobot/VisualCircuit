import os
import numpy as np
from utils.wires.wire_str import share_string

# Accepts a list of strings as a message

def send_message(message):

	port = str(os.getpid())
	sh_mem = share_string(port) 
	to_send = np.array(message, dtype='<U64')
	sh_mem.add(to_send)
	
