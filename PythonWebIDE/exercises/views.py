from django.shortcuts import render
from django.http import HttpResponse, JsonResponse
from django.conf import settings
from .models import Exercise
import ast
import json
import os
import shutil


from django.http import JsonResponse

import os

#TODO: Too many hardcoded strings, review

last_path = 0

last_project_url = ""

def index(request):
    projects = get_directories_in_folder("FileSystem")
    for i, p in enumerate(projects):
        projects[i] = p[11:].replace("\\", "/")
    context = {"projects": projects}
    return render(request, 'exercises/RoboticsAcademy.html', context)


def load_exercise(request, exercise_id, project_url):
    exercise = Exercise.objects.get(exercise_id=exercise_id)
    print(project_url)
    global last_project_url
    global last_path
    last_project_url = project_url
    last_path = 0
    try:
        with open(project_url+"/metadata/metadata.json", 'r') as f:
            data = json.load(f)
        newContext = exercise.context.copy()
        if data['last_world'] is not None:
            newContext["exercise_config"]["launch"]["0"]["launch_file"] = '/workspace/worlds/general_launch_file.launch'
        return render(request, 'exercises/' + exercise_id + '/exercise.html', newContext)
    except:
        print("metadata file was empty or in an invalid format")
    return render(request, 'exercises/' + exercise_id + '/exercise.html', exercise.context)


def request_code(request, exercise_id):
    difficulty = request.GET.get('diff')
    path = '/exercises/static/exercises/{}/assets/{}.py'.format(exercise_id, difficulty)
    path = str(settings.BASE_DIR) + path
    print('PATH: ', path)
    with open(path) as f:
        data = f.read().replace('\\n', '\n')

    print(data)

    if difficulty != None:
        print('EXERCISE: ', exercise_id, 'DIFFICULTY: ', difficulty)
        return HttpResponse(data, content_type="text/plain")

def get_folder_content(request, folder_path):
    files = []
    for (dirpath, dirnames, filenames) in os.walk(folder_path):
        for file in filenames:
            file_path = os.path.join(dirpath, file)
            files.append(file_path.replace(folder_path, '', 1))

    return JsonResponse({'files': files})


def get_directories_in_folder(folder_path):
    directories = []
    for root, dirs, files in os.walk(folder_path):
        if root == folder_path:
            continue
        # limit search to two levels deep
        if root.count(os.sep) - folder_path.count(os.sep) > 1:
            del dirs[:]
            continue
        # append all directories in the current level
        for dir in dirs:
            directories.append(os.path.join(root, dir))
    return directories


def get_file_content(request, file_path):
    with open(file_path, 'r') as file:
        data = file.read()

    return HttpResponse(data, content_type="text/plain")

def save_file_content(request, file_path):
    global last_path
    print(last_path)

    if not file_path.lower().endswith(('.py', '.js', '.txt', '.css', '.json', '.world')):

        return HttpResponse("Archivo no guardable", content_type="text/plain")

    if(last_path != 0):
        data = json.loads(request.body.decode('utf-8'))
        editor_code = data.get('data')
        with open(last_path, 'w') as file:
            file.write(editor_code)
        last_path = file_path
        return HttpResponse("Se ha guardado", content_type="text/plain")

    last_path = file_path

    return HttpResponse("No se ha podido guardar", content_type="text/plain")

def save_data(request):
    if(last_path != 0):
        data = json.loads(request.body.decode('utf-8'))
        editor_code = data.get('data')
        with open(last_path, 'w') as file:
            file.write(editor_code)

        return HttpResponse("guardado", content_type="text/plain")

    return HttpResponse("no guardado", content_type="text/plain")

def add_world_file(request):
    data = json.loads(request.body.decode('utf-8'))
    url = data.get('url')
    value = data.get('value')
    name = data.get('name')
    with open(os.path.join(url, name+".world"), "w") as file:
        file.write(value)
    return HttpResponse("Se ha guardado correctamente", content_type="text/plain")

def save_metadata_file(request):
    data = json.loads(request.body.decode('utf-8'))
    newWorld = data.get('data')
    url = data.get('url')
    name = data.get('name')
    size = os.path.getsize(url+"metadata.json") == 0
    if(size):
        metadata = {}
    else:
        with open(url+"metadata.json", 'r') as file:
            metadata = json.load(file)
    metadata["last_world"] = newWorld
    metadata["last_world_name"] = name + ".world"
    with open(url+"metadata.json", 'w') as file:
        json.dump(metadata, file)

    return HttpResponse("Se ha actualizado el último mundo usado", content_type="text/plain")


def get_project_url(request):
    print(last_project_url)
    return HttpResponse(last_project_url, content_type="text/plain")


def load_file_into_directory(request):
    data = json.loads(request.body.decode('utf-8'))
    body = data.get('body')
    name = data.get('name')
    full_name = name.split(".")
    if full_name[1] != "world":
        with open(last_project_url + "/application/" + name, 'w') as file:
            file.write(body)
    else:
        with open(last_project_url + "/world/" + name, 'w') as file:
            file.write(body)
    return HttpResponse("Se ha cargado el archivo", content_type="text/plain")

def copy_file_into_directory(request):
    data = json.loads(request.body.decode('utf-8'))
    path = data.get('path')
    original = last_project_url + path
    full_path = path.split(".");
    target = last_project_url + full_path[0] + " - copy." + full_path[1]

    print(original)
    print(target)

    shutil.copyfile(original, target)

    return HttpResponse("Se ha copiadso el archivo", content_type="text/plain")

def delete_file_into_directory(request):
    global last_path
    data = json.loads(request.body.decode('utf-8'))
    path = data.get('path')
    original = last_project_url + path

    os.remove(original)

    last_path = 0

    return HttpResponse("Se ha borrado el archivo", content_type="text/plain")

def get_files_content(request,  folder_path):
    files = []
    for (dirpath, dirnames, filenames) in os.walk(folder_path):
        for file in filenames:
            file_path = os.path.join(dirpath, file)
            file_path = file_path.replace('\\', '/')
            with open(file_path, 'r') as f:
                data = f.read()
            files.append([file, data])

    print(files)
    return JsonResponse({'files': files})

def create_empty_file(request, file_path):
    open(file_path, 'w')

    return HttpResponse(content_type="text/plain")

def rename_file(request, file_path):
    data = json.loads(request.body.decode('utf-8'))
    old_name = data.get('oldName')
    new_name = data.get('newName')

    if os.path.isfile(file_path + '/' + new_name):
        print("The file already exists")
    else:
        # Rename the file
        os.rename(file_path + '/' + old_name, file_path + '/' + new_name)
    return HttpResponse(content_type="text/plain")
