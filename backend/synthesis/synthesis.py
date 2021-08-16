from io import BytesIO
from typing import Tuple
from pathlib import Path
from synthesis.file_utils import InMemoryZip
from django.contrib.staticfiles.storage import staticfiles_storage
from django.contrib.staticfiles.utils import get_files
import json


BLOCK_DIRECTORY = 'modules'


def syntheize_modules(data: dict, zipfile: InMemoryZip) -> InMemoryZip:
    '''Synthesize python code for different blocks as well as user code blocks.
    Different blocks present in the project is collected.
    Parameters of each dependency block as well as constant blocks are collected.
    Blocks, parameters and the connections (wires) between the blocks stored in a 
    JSON file.
    '''
    dependencies = {}
    blocks = {}
    parameters = {}

    for key, dependency in data['dependencies'].items():
        components = dependency['design']['graph']['blocks']
        for block in components:
            if block['type'] == 'basic.code':
                # If code, generate python file.
                script = block['data']['code']
                script_name = dependency['package']['name']
                script_name += dependency['package']['version'].replace('.', '')
                dependencies[key] = script_name
                zipfile.append(f'{BLOCK_DIRECTORY}/{script_name}.py', script)
            elif block['type'] == 'basic.constant':
                # If constant, it is a parameter.
                # Since this is a parameter for a dependency block we make the dependency type block
                # as the key. This is later used to identify which parameter belongs to which dependency
                # block
                parameters[key] = parameters.get(key, [])
                parameters[key].append({
                    'id': block['id'],
                    'name': block['data']['name'], 
                    'value': block['data']['value']}
                )

    count = 1
    for block in data['design']['graph']['blocks']:
        block_id, block_type = block['id'], block['type']
        if block_type == 'basic.code':
            # If code, generate python file.
            code_name = "Code_"+str(count)
            count += 1
            script = block['data']['code']
            zipfile.append(f'{BLOCK_DIRECTORY}/{code_name}.py', script)
            blocks[block_id] = {'name': code_name, 'type': block_type}
        elif block_type == 'basic.constant':
            # If constant, it is a parameter.
            # Since this is a parameter at project level, we make the key as constant block ID
            parameters[block_id] = {'name': block['data']['name'], 'value': block['data']['value']}
        else:
            # TODO: Check how input and output blocks behave.
            # This behaviour is for Package blocks only
            blocks[block_id] = {'name': dependencies[block_type], 'type': block_type}


    data = {'blocks': blocks, 'parameters': parameters, 'wires': data['design']['graph']['wires']}
    zipfile.append('data.json', json.dumps(data))

    return zipfile

def synthesize_executioner(zipfile: InMemoryZip) -> InMemoryZip:
    '''Synthesize python code necessary to run the blocks.
    All these files are present in django static directory. 
    They are read and put into the zipfile.
    '''
    for path in get_files(staticfiles_storage, location='synthesis'):
        with staticfiles_storage.open(path) as file:
            content = file.read()
            relative_path = Path(path).relative_to('synthesis')
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
    zipfile = synthesize_executioner(zipfile)
    zipfile = syntheize_modules(data, zipfile)
    zipfile = syntesize_extras(zipfile)

    # Project name (zipfile name) 
    project_name = f"{data['package']['name']}.zip" if data['package']['name'] != '' else 'Project.zip'

    return project_name, zipfile.get_zip()