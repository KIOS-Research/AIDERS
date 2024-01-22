from django.core.management.base import BaseCommand

from aiders import models


class Command(BaseCommand):
    help = "Reset Database clients."

    def handle(self, *args, **options):
        for drone in models.Drone.objects.all():
            drone.is_connected_with_platform = False
            drone.build_map_activated = False
            drone.save()
        models.Detection.objects.all().update(
            detection_status=models.Detection.DetectionStatusChoices.DETECTION_DISCONNECTED
        )
        for device in models.Device.objects.all():
            device.is_connected_with_platform = False
            device.save()
        for baloraMaster in models.BaloraMaster.objects.all():
            baloraMaster.is_connected_with_platform = False
            baloraMaster.save()
