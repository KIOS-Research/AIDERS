import contextlib
from django.apps import apps
from django.contrib import admin

# Get models only from the app
app_config = apps.get_app_config('aiders')
models = app_config.get_models()

for model in models:
    # Skip models that are already registered
    if admin.site.is_registered(model):
        continue

    # Attempt to register the model
    with contextlib.suppress(admin.sites.AlreadyRegistered):
        admin.site.register(model)