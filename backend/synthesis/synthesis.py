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


    def process_wires(valid_wires):
        count = 0  # Initialize a counter for processing iterations
        changes_detected = True  # Boolean flag to track if changes were made in the current iteration

        # Loop until no changes are detected
        while changes_detected:
            wire_check_source = {}  # Dictionary to store wires' source info where 'ob' is absent
            wire_check_target = {}  
            changes_detected = False  # Reset changes_detected to False before processing wires

            new_wires = []  # List to store newly created wires

            # Iterate through valid_wires in reverse order
            for i in range(len(valid_wires) - 1, -1, -1):
                wire = valid_wires[i]  # Access the current wire from valid_wires
                remove_wire = False  # Flag to mark whether the current wire should be removed

                count += 1  # Increment the processing counter

                # Check if the source port is 'input-out'
                if wire['source']['port'] == 'input-out':
                    port_name = wire['source']['block']  # Get the block name of the source port
                    
                    # If the target has 'ob' and it's 'absent', track it for processing later
                    if 'ob' in wire['target'] and wire['target']['ob'] == 'absent':
                        if port_name not in wire_check_source:  
                            wire_check_source[port_name] = []  # Initialize list if it's not present
                        wire_check_source[port_name].append(wire['target'].copy())  # Append a copy of the target
                        wire_check_source[port_name][-1]['port'] = port_name  # Set the port name for the new entry
                    else:
                        # Otherwise, add the target to wire_check_source and mark for removal
                        if port_name not in wire_check_source:  
                            wire_check_source[port_name] = []
                        wire_check_source[port_name].append(wire['target'])  # Add target to the dictionary
                        remove_wire = True  # Mark the wire for removal later

                # Check if the target port is 'output-in'
                if wire['target']['port'] == 'output-in':
                    port_name = wire['target']['block']  # Get the block name of the target port
                    
                    # If the source has 'ob' and it's 'absent', track it for processing later
                    if 'ob' in wire['source'] and wire['source']['ob'] == 'absent':
                        if port_name not in wire_check_target:  
                            wire_check_target[port_name] = []  # Initialize list if it's not present
                        wire_check_target[port_name].append(wire['source'].copy())  # Append a copy of the source
                        wire_check_target[port_name][-1]['port'] = port_name  # Set the port name for the new entry
                    else:
                        # Otherwise, add the source to wire_check_target and mark for removal
                        if port_name not in wire_check_target:
                            wire_check_target[port_name] = []
                        wire_check_target[port_name].append(wire['source'])  # Add source to the dictionary
                        remove_wire = True  # Mark the wire for removal later

                # Remove the wire if it was marked for removal
                if remove_wire:
                    del valid_wires[i]  # Remove the wire from valid_wires
                    changes_detected = True  # Set changes_detected to True as wires were modified

            i = 0  # Initialize counter for iterating through valid_wires
            new_wires = []  # Reset the list to store any newly created wires

            # Iterate through valid_wires starting from the beginning
            while i < len(valid_wires):
                wire = valid_wires[i]  # Access the current wire

                # Check if the source port is exactly 36 characters (for specific port length)
                if len(wire['source'].get('port', '')) == 36:
                    port_name = wire['source']['port']  # Get the port name of the source

                    # If the port name exists in wire_check_target, process the sources
                    if port_name in wire_check_target and wire_check_target[port_name]:
                        new_sources = wire_check_target[port_name]  # Get the list of sources for the port
                        
                        # Replace the source of the current wire with the first new source
                        valid_wires[i]['source'] = new_sources[0]
                        if 'ob' in valid_wires[i]['source']:  # If 'ob' exists, set the port name
                            valid_wires[i]['source']['port'] = port_name
                        
                        # For remaining sources, create new wires
                        for new_source in new_sources[1:]:
                            new_wire = wire.copy()  # Copy the current wire
                            new_wire['source'] = new_source  # Set the new source for the copied wire
                            if 'ob' in new_wire['source']:  # If 'ob' exists, set the port name
                                new_wire['source']['port'] = port_name
                            new_wires.append(new_wire)  # Add the new wire to new_wires

                # Check if the target port is exactly 36 characters (for specific port length)
                if len(wire['target'].get('port', '')) == 36:
                    port_name = wire['target']['port']  # Get the port name of the target
                    
                    # If the port name exists in wire_check_source, process the targets
                    if port_name in wire_check_source and wire_check_source[port_name]:
                        new_targets = wire_check_source[port_name]  # Get the list of targets for the port
                        
                        # Replace the target of the current wire with the first new target
                        valid_wires[i]['target'] = new_targets[0]
                        if 'ob' in valid_wires[i]['target']:  # If 'ob' exists, set the port name
                            valid_wires[i]['target']['port'] = port_name
                        
                        # For remaining targets, create new wires
                        for new_target in new_targets[1:]:
                            new_wire = wire.copy()  # Copy the current wire
                            new_wire['target'] = new_target  # Set the new target for the copied wire
                            if 'ob' in new_wire['target']:  # If 'ob' exists, set the port name
                                new_wire['target']['port'] = port_name
                            new_wires.append(new_wire)  # Add the new wire to new_wires

                i += 1  # Move to the next wire in the valid_wires list

            # Append newly created wires to the valid_wires list
            valid_wires.extend(new_wires)

            # Remove duplicate wires by checking for uniqueness
            unique_wires = []  # List to store unique wires
            seen_wires = set()  # Set to track already seen wires

            # Iterate over valid_wires to remove duplicates
            for wire in valid_wires:
                # Convert the wire dictionary to a frozenset for hashability (so it can be added to a set)
                wire_tuple = frozenset((key, frozenset(value.items())) if isinstance(value, dict) else (key, value) for key, value in wire.items())
                
                # Only add unique wires to unique_wires
                if wire_tuple not in seen_wires:
                    seen_wires.add(wire_tuple)  # Add the wire to the set of seen wires
                    unique_wires.append(wire)  # Append the wire to unique_wires

            valid_wires = unique_wires  # Assign the unique wires back to valid_wires

        return valid_wires  # Return the processed valid_wires


    # Call the process_wires function to process the wires
    processed_wires = process_wires(valid_wires)

    # Package processed data into a JSON file for saving or further use
    data = {
        'blocks': blocks,  # Block-related data
        'parameters': parameters,  # Parameter-related data
        'synchronize_frequency': synhronize_frequency,  # Synchronization frequency information
        'wires': processed_wires  # The processed wire data
    }

    # Add data to a zipfile (the zipfile object must be defined elsewhere in the code)
    zipfile.append('data.json', json.dumps(data))  # Append the JSON data to the zipfile


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