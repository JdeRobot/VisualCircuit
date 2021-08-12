from django.urls import path
from . import views

urlpatterns = [
    path('json', views.index, name='json'),
    path('download', views.download, name='download'),
]
