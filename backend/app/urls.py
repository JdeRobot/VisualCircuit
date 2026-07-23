from django.urls import path
from . import views

urlpatterns = [
    path('build', views.build, name='build'),
    path('download', views.download, name='download'),
    path('install_block', views.install_block, name='install_block'),
    path('installed_blocks', views.installed_blocks, name='installed_blocks'),
]
