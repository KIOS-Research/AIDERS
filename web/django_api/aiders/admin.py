import contextlib
from django.apps import apps
from django.contrib import admin

models = apps.get_models()

for model in models:
    with contextlib.suppress(admin.sites.AlreadyRegistered):
        admin.site.register(model)
