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
    wire_comp = data['design']['graph']['wires']
    dep_no = {}  # Dictionary to store the number of blocks of each type

    def process_dependency(dep, zipfile, synhronize_frequency, optional_files, dep_no, parameters, dependencies):
        
        for key, dependency in dep.items():
            components = dependency['design']['graph']['blocks']
            print("dependency['design']['graph']['blocks']",dependency['design']['graph']['blocks'])
            wire_comp.extend(dependency['design']['graph']['wires'])
            for block in components:
                block_id = block['id']
                if block['type'] == 'basic.code':
                    script = block['data']['code']
                    script_name = dependency['package']['name']
                    synhronize_frequency[key] = get_number_or_default(block['data'].get('frequency', 30), 30)
                    
                    if script_name in optional_files:
                        optional_files[script_name] = True

                    script_name += dependency['package']['version'].replace('.', '')
                    
                    if script_name in dep_no:
                        dep_no[script_name] += 1
                    else:
                        dep_no[script_name] = 1

                    script_name += str(dep_no[script_name])
                    dependencies[key] = script_name

                    zipfile.append(f'{BLOCK_DIRECTORY}/{script_name}.py', script)
                    blocks[block_id] = {'name': script_name, 'type': block_type}

                elif block['type'] == 'basic.constant':
                    parameters[key] = parameters.get(key, [])
                    parameters[key].append({
                        'id': block['id'],
                        'name': block['data']['name'], 
                        'value': block['data']['value']}
                    )

            if 'dependencies' in dependency and dependency['dependencies']:
                process_dependency(dependency['dependencies'], zipfile, synhronize_frequency, optional_files, dep_no, parameters, dependencies)

    # Process the top-level dependencies
    if 'dependencies' in data:
        process_dependency(data['dependencies'], zipfile, synhronize_frequency, optional_files, dep_no, parameters, dependencies)

    count = 1
    print("depend ba",data['design']['graph']['blocks'])
    for block in data['design']['graph']['blocks']:
        print("depend b",dependencies)
        print("block intern",block)
        if 'source' not in block and 'target' not in block:
            block_id, block_type = block['id'], block['type']
            if block_type == 'basic.code':
                code_name = "Code_" + str(count)
                count += 1
                script = block['data']['code']
                synhronize_frequency[block_id] = get_number_or_default(block['data'].get('frequency', 30), 30)
                zipfile.append(f'{BLOCK_DIRECTORY}/{code_name}.py', script)
                blocks[block_id] = {'name': code_name, 'type': block_type}
            elif block_type == 'basic.constant':
                parameters[block_id] = [{'name': block['data']['name'], 'value': block['data']['value']}]
            else:
                blocks[block_id] = {'name': dependencies.get(block_type, ''), 'type': block_type}
                print("blocks[block_id]",blocks[block_id])
                print("depend",dependencies)
    print("wire_comp",wire_comp)
    
    data = {
        'blocks': blocks, 
        'parameters': parameters, 
        'synchronize_frequency': synhronize_frequency, 
        'wires': wire_comp
    }
    zipfile.append('data.json', json.dumps(data))

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