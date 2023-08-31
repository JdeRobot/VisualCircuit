from django.urls import path

from . import views

urlpatterns = [
    path('', views.index, name='index'),
    path("exercises/<slug:exercise_id>/<path:project_url>", views.load_exercise, name="load_exercise"),
    path("exercise/request/<slug:exercise_id>", views.request_code, name="request_code"),
    path('exercise/get_folder_content/<path:folder_path>/', views.get_folder_content, name="get_folder_content"),
    path('exercise/get_file_content/<path:file_path>/', views.get_file_content, name="get_file_content"),
    path('exercise/save_file_content/<path:file_path>/', views.save_file_content, name="save_file_content"),
    path('exercise/save_data/', views.save_data, name="save_data"),
    path('exercise/add_world_file/', views.add_world_file, name="add_world_file"),
    path('exercise/save_metadata_file/', views.save_metadata_file, name="save_metadata_file"),
    path('exercise/get_project_url/', views.get_project_url, name="get_project_url"),
    path('exercise/load_file_into_directory/', views.load_file_into_directory, name="load_file_into_directory"),
    path('exercise/copy_file_into_directory/', views.copy_file_into_directory, name="copy_file_into_directory"),
    path('exercise/delete_file_into_directory/', views.delete_file_into_directory, name="delete_file_into_directory"),
    path('exercise/get_files_content/<path:folder_path>/', views.get_files_content, name="get_files_content"),
    path('exercise/create_empty_file/<path:file_path>/', views.create_empty_file, name="create_empty_file"),
    path('exercise/rename_file/<path:file_path>/', views.rename_file, name="rename_file"),
]
