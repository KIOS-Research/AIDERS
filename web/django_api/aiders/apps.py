from django.apps import AppConfig


class MyAppConfig(AppConfig):
    name = "aiders"
    verbose_name = "Aiders"
    def ready(self):
        import aiders.signals  # Ensure signals are connected