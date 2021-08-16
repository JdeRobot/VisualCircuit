import os
import numpy as np
from time import sleep
import threading
from utils.wires.wire_str import share_string as ww

'''
 name contains the block name
 control_data[0] contains number of ticks
 control_data[1] contains calculated delay value
'''

def monitor_frequency(name, control_data, required_frequency, update):

    monitoring_thread = threading.Thread(target=frequency_monitor, args=(name, control_data, required_frequency, update, ))
    monitoring_thread.start()
    
def frequency_monitor(name, control_data, required_frequency, update):

    port = str(os.getpid())
    sh_mem = ww(port)
    
    sleep_time = 5.0
    
    # Give enough time to get estimates
    sleep(sleep_time)
    
    measured_frequency = (control_data[0]/sleep_time) 
    exec_time = 1.0/measured_frequency - control_data[1]
    avg_exec_time = exec_time
  
    while(True):
    
        sleep(sleep_time)
        
        measured_frequency = control_data[0]/sleep_time
        avg_exec_time += (1.0/measured_frequency)-control_data[1]
        avg_exec_time /= 2.0
        
        # We don't want the delay value to change if a block is disabled.
        if (measured_frequency > required_frequency+1 or measured_frequency < required_frequency-1) and update != 0:
            control_data[1] = 1.0 - (avg_exec_time*required_frequency) #Delay value
            control_data[1] = control_data[1]/required_frequency
        
        if control_data[1] < 0:
            control_data[1] = 0
        
        control_data[0] = 0
        message = [name , 'Frequency: '+str(measured_frequency)+'  CPU Time: '+str(round(avg_exec_time,3)*1000)+'ms']
        to_send = np.array(message, dtype='<U64')
        sh_mem.add(to_send)
