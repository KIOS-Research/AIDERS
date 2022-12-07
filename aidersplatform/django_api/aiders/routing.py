from django.urls import path
from .consumers import *

ws_urlpatterns =[
    path('ws/index/', ws_index.as_asgi()),
    path('ws/monitoring/', ws_monitoring.as_asgi()),
]