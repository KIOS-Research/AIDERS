from django.contrib.gis.db import models


class SystemMonitoring(models.Model):
    time = models.DateTimeField(auto_now=True)
    cpu_usage = models.FloatField()
    cpu_core_usage = models.CharField(max_length=255, blank=False)
    cpu_temp = models.FloatField()
    gpu_usage = models.FloatField(blank=True, null=True)
    gpu_memory = models.FloatField(blank=True, null=True)
    gpu_temp = models.FloatField(blank=True, null=True)
    ram_usage = models.FloatField()
    swap_memory_usage = models.FloatField()
    temp = models.FloatField()
    upload_speed = models.FloatField()
    download_speed = models.FloatField()
    total_network = models.FloatField()
    disk_read = models.FloatField()
    disk_write = models.FloatField()
    battery_percentage = models.FloatField(blank=True, null=True)
