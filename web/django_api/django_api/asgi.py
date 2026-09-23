"""
ASGI config for django_api project.

It exposes the ASGI callable as a module-level variable named ``application``.

For more information on this file, see
https://docs.djangoproject.com/en/3.2/howto/deployment/asgi/
"""

import os

from channels.auth import AuthMiddlewareStack
from channels.routing import ProtocolTypeRouter, URLRouter
from django.core.asgi import get_asgi_application

os.environ.setdefault("DJANGO_SETTINGS_MODULE", "django_api.settings")

# The app registry must be populated before the consumers (which import models
# and views) can be imported, so get_asgi_application() has to run first.
# runserver already did this for us, standalone servers like daphne did not.
django_asgi_app = get_asgi_application()

from aiders.routing import ws_urlpatterns  # noqa: E402

application = ProtocolTypeRouter(
    {
        "http": django_asgi_app,
        "websocket": AuthMiddlewareStack(URLRouter(ws_urlpatterns)),
    }
)
