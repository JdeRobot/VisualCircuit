from django.urls import path
from . import views

urlpatterns = [
    path('json', views.index, name='download'),
    path('download', views.download, name='download'),
]
