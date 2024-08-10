from io import BytesIO
from typing import Dict, Tuple
from pathlib import Path
from synthesis.file_utils import InMemoryZip
from django.contrib.staticfiles.storage import staticfiles_storage
from django.contrib.staticfiles.utils import get_files
import json

BLOCK_DIRECTORY = 'modules'

OPTIONAL_FILES = {
    'FaceDetector' : 'utils/models/haar_cascade/*',
    'ObjectDetector': 'utils/models/yolov3/*'
}

PROJECT_FILE_EXTENSION = '.vc3'

def get_number_or_default(num, default):
    try:
        num = float(num)
        return num
    except ValueError:
        return default

def syntheize_modules(data: dict, zipfile: InMemoryZip) -> Tuple[InMemoryZip, Dict[str, bool]]:
    '''Synthesize python code for different blocks as well as user code blocks.
    Different blocks present in the project are collected.
    Parameters of each dependency block as well as constant blocks are collected.
    Blocks, parameters and the connections (wires) between the blocks stored in a 
    JSON file.
    '''
    # Initialize an empty dictionary for tracking dependencies, blocks, parameters, synhronize_frequency, optional_files
    dependencies = {}
    blocks = {}
    parameters = {}
    synhronize_frequency = {}
    optional_files = {}
    wire_comp = data['design']['graph']['wires'] # Retrieve all wire connections from the design graph
    dep_no = {}  # Dictionary to store the number of blocks of each type

    # Function to process dependencies and extract block information
    def process_dependency(dep, zipfile, synhronize_frequency, optional_files, dep_no, parameters, dependencies):
        # Iterate over each dependency
        for key, dependency in dep.items():
            
            components = dependency['design']['graph']['blocks']    # Retrieve all blocks in the dependency
            wire_comp.extend(dependency['design']['graph']['wires'])    # Add all wires from the dependency to the main wire list
            # Iterate over each block in the dependency
            for block in components:

                block_id = block['id']
                block_type = block['type']

                # Check if the block is of type 'basic.code'
                if block_type == 'basic.code':
                    
                    script = block['data']['code']  # Retrieve the script code from the block's data
                    script_name = dependency['package']['name'] # Retrieve the script name from the dependency package
                    synhronize_frequency[block_id] = get_number_or_default(block['data'].get('frequency', 30), 30)  # Set the synchronization frequency, defaulting to 30
                    
                    # Check if the script name is already in optional files
                    if script_name in optional_files:
                        optional_files[script_name] = True  # Mark the script as required in optional files

                    script_name += dependency['package']['version'].replace('.', '')    # Append version number to the script name, removing dots
                    
                    # Increment the count for this script type or initialize it
                    if script_name in dep_no:
                        dep_no[script_name] += 1
                    else:
                        dep_no[script_name] = 1

                    
                    script_name += str(dep_no[script_name])     # Add the block count to the script name
                    dependencies[key] = script_name # Map the key to the script name in dependencies
                    
                    
                    zipfile.append(f'{BLOCK_DIRECTORY}/{script_name}.py', script)   # Add the script to the zipfile
                    blocks[block_id] = {'name': script_name, 'type': block_type}    # Store block information in the blocks dictionary

                # Check if the block is of type 'basic.constant'
                elif block_type == 'basic.constant':
                    parameters[block_id] = parameters.get(block_id, []) # Initialize parameter list for the block if not already present

                    # Append block's parameter data to the parameters dictionary
                    parameters[block_id].append({
                        'id': block['id'],
                        'name': block['data']['name'], 
                        'value': block['data']['value']}
                    )

            # Recursively process nested dependencies if present
            if 'dependencies' in dependency and dependency['dependencies']:
                process_dependency(dependency['dependencies'], zipfile, synhronize_frequency, optional_files, dep_no, parameters, dependencies)

    # Process the top-level dependencies from the data
    if 'dependencies' in data:
        process_dependency(data['dependencies'], zipfile, synhronize_frequency, optional_files, dep_no, parameters, dependencies)

    
    count = 1   # Initialize a counter for naming blocks


    # Iterate over the blocks in the main design graph
    for block in data['design']['graph']['blocks']:
        
        if 'source' not in block and 'target' not in block: # Skip blocks that have a 'source' or 'target' property
            
            block_id, block_type = block['id'], block['type']
            # Check if the block is of type 'basic.code'
            if block_type == 'basic.code':
                
                code_name = "Code_" + str(count)    # Generate a unique code name for the block
                count += 1  # Increment the block counter
                script = block['data']['code']  # Retrieve the script code from the block's data

                # Set the synchronization frequency, defaulting to 30
                synhronize_frequency[block_id] = get_number_or_default(block['data'].get('frequency', 30), 30)
                zipfile.append(f'{BLOCK_DIRECTORY}/{code_name}.py', script) # Add the script to the zipfile with the generated code name
                blocks[block_id] = {'name': code_name, 'type': block_type}  # Store block information in the blocks dictionary

            # Check if the block is of type 'basic.constant'
            elif block_type == 'basic.constant':
                # Add block's parameter data to the parameters dictionary
                parameters[block_id] = [{'name': block['data']['name'], 'value': block['data']['value']}]

    
    valid_wires = []    # Initialize a list to store valid wire connections


    # Iterate over all wires in the wire component list
    for wire in wire_comp:

        # Retrieve source and target block IDs for the wire
        source_id = wire['source']['block']
        target_id = wire['target']['block']

        # Check if the source and target blocks are in the blocks or parameters dictionary
        source_in_blocks = source_id in blocks
        source_in_parameters = source_id in parameters
        target_in_blocks = target_id in blocks
        target_in_parameters = target_id in parameters

        # Validate wires based on presence in blocks or parameters dictionaries
        if (source_in_blocks and target_in_blocks) or \
        (source_in_parameters and target_in_parameters) or \
        (source_in_blocks and target_in_parameters) or \
        (source_in_parameters and target_in_blocks):
            # If valid, add the wire to the valid_wires list
            valid_wires.append(wire)
        else:
            # Mark source and target as 'absent' if not in blocks or parameters
            if source_id not in blocks and source_id not in parameters:
                wire['source']['ob'] = 'absent'
            if target_id not in blocks and target_id not in parameters:
                wire['target']['ob'] = 'absent'
            # Add the wire to the valid_wires list
            valid_wires.append(wire)

    # Function to process wires and handle absent blocks
    def process_wires(valid_wires):
        # Initialize dictionaries to track source and target ports
        wire_check_source = {}
        wire_check_target = {}

        # Initialize a counter for iteration
        count = 0
        # Set a flag to detect changes
        changes_detected = True

        # Continue processing while changes are detected
        while changes_detected:
            changes_detected = False

            # Iterate over valid wires in reverse order
            for i in range(len(valid_wires) - 1, -1, -1):
                wire = valid_wires[i]
                remove_wire = False # Flag to determine if the wire should be removed
                
                count += 1  # Increment the counter

                # Check if the source port is 'input-out'
                if wire['source']['port'] == 'input-out':
                    port_name = wire['source']['block']
                    # Check if the target is marked 'absent'
                    if 'ob' in wire['target'] and wire['target']['ob'] == 'absent':
                        # Add target details to wire_check_source with 'input-out' port
                        wire_check_source[port_name] = wire['target'].copy()
                        wire_check_source[port_name]['port'] = port_name
                    else:
                        # Add target details to wire_check_source
                        wire_check_source[port_name] = wire['target']
                        # Mark the wire for removal
                        remove_wire = True

                # Check if the target port is 'output-in'
                if wire['target']['port'] == 'output-in':
                    port_name = wire['target']['block']
                    # Check if the source is marked 'absent'
                    if 'ob' in wire['source'] and wire['source']['ob'] == 'absent':
                        # Add source details to wire_check_target with 'output-in' port
                        wire_check_target[port_name] = wire['source'].copy()
                        wire_check_target[port_name]['port'] = port_name
                    else:
                        # Add source details to wire_check_target
                        wire_check_target[port_name] = wire['source']
                        # Mark the wire for removal
                        remove_wire = True

                # Remove the wire from valid_wires if marked for removal
                if remove_wire:
                    del valid_wires[i]
                    changes_detected = True # Set flag to true since changes were made

            # Iterate over the valid wires to update ports
            for i, wire in enumerate(valid_wires):
                # Check if the source port name has 36 characters 
                if len(wire['source'].get('port', '')) == 36:
                    port_name = wire['source']['port']
                    # Update source with wire_check_target details if present
                    if port_name in wire_check_target:
                        valid_wires[i]['source'] = wire_check_target[port_name]
                        # Update port name if 'absent' flag is present
                        if 'ob' in valid_wires[i]['source']:
                            valid_wires[i]['source']['port'] = port_name

                # Check if the target port name has 36 characters 
                if len(wire['target'].get('port', '')) == 36:
                    port_name = wire['target']['port']
                    # Update target with wire_check_source details if present
                    if port_name in wire_check_source:
                        valid_wires[i]['target'] = wire_check_source[port_name]
                        # Update port name if 'absent' flag is present
                        if 'ob' in valid_wires[i]['target']:
                            valid_wires[i]['target']['port'] = port_name

        # Return the processed list of valid wires
        return valid_wires

    # Process the wires to handle 'absent' blocks
    processed_wires = process_wires(valid_wires)

    # Create a final data dictionary with blocks, parameters, frequencies, and wires
    data = {
        'blocks': blocks, 
        'parameters': parameters, 
        'synchronize_frequency': synhronize_frequency, 
        'wires': processed_wires
    }
    # Add the data dictionary as a JSON file in the zipfile
    zipfile.append('data.json', json.dumps(data))

    # Return the updated zipfile and optional files dictionary
    return zipfile, optional_files

    
def synthesize_executioner(zipfile: InMemoryZip, optional_files: Dict[str, bool]) -> InMemoryZip:
    '''Synthesize python code necessary to run the blocks.
    All these files are present in django static directory. 
    They are read and put into the zipfile.
    '''

    # Blacklist all optional files by default
    paths_to_exclude = set(OPTIONAL_FILES.values())
    # If a particular block is present which needs optional files, whitelist the required optional files.
    paths_to_include = set([path for key, path in OPTIONAL_FILES.items() if optional_files.get(key, False)])
    
    for path in get_files(staticfiles_storage, location='synthesis'):
        with staticfiles_storage.open(path) as file:
            content = file.read()
            relative_path = Path(path).relative_to('synthesis')
            # Check if the path is excluded, if it is, check if its required for the current set of blocks.
            if not any([relative_path.match(p) for p in paths_to_exclude]) or any([relative_path.match(p) for p in paths_to_include]):
                zipfile.append(str(relative_path), content)       

    return zipfile

def syntesize_extras(zipfile: InMemoryZip) -> InMemoryZip:
    '''Create and extra files which might be required for execution.
    '''
    zipfile.append('logs/console.log', '')
    return zipfile

def synthesize(data: dict) -> Tuple[str, BytesIO]:
    '''Synthesize a python application corresponding to the VC project file.
    All synthesized files are put inside a zip file so that it can be downloaded.
    '''
    zipfile = InMemoryZip()
    # Optional files required based on blocks present.
    zipfile, optional_files = syntheize_modules(data, zipfile)
    zipfile = synthesize_executioner(zipfile, optional_files)
    zipfile = syntesize_extras(zipfile)

    # Project name (zipfile name) 
    project_name = f"{data['package']['name']}" if data['package']['name'] != '' else 'Project'

    # Add the .vc3 file to the built application, this will let us easily load the project in Visual Circuit
    zipfile.append(project_name + PROJECT_FILE_EXTENSION, json.dumps(data))
    # .zip is required for the name of the full package
    project_name += '.zip'

    return project_name, zipfile.get_zip()