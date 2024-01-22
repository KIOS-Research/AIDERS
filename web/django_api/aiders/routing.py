from django.urls import path
from .consumers import *

ws_urlpatterns = [
    path("ws/platform/", ws_platform.as_asgi()),
    path("ws/monitoring/", ws_monitoring.as_asgi()),
]
