from django.contrib.gis.db import models
from django.utils import timezone

from .operation import Operation


class GroundVehicle(models.Model):
    """Model for tracking ground vehicles in operations"""
    
    name = models.CharField(max_length=100, unique=True)
    model = models.CharField(max_length=200)
    service = models.CharField(max_length=100, blank=True, null=True)
    lat = models.FloatField(null=True, blank=True)
    lon = models.FloatField(null=True, blank=True)
    speed = models.FloatField(default=0.0, help_text="Speed in km/h")
    heading = models.FloatField(default=0.0, help_text="Heading in degrees (0-360)")
    status = models.CharField(max_length=100, default="Unknown")
    last_updated = models.DateTimeField(auto_now=True)
    operation = models.ForeignKey(Operation, on_delete=models.SET_NULL, blank=True, null=True)
    
    class Meta:
        ordering = ['-last_updated']
        verbose_name = "Ground Vehicle"
        verbose_name_plural = "Ground Vehicles"
    
    def __str__(self):
        return f"{self.name} ({self.model})"
    
    @staticmethod
    def getGroundVehicleIdByName(name):
        """Get ground vehicle ID by name"""
        try:
            return GroundVehicle.objects.get(name=name).id
        except GroundVehicle.DoesNotExist:
            return None
    
    @staticmethod
    def getGroundVehicleNameById(pk):
        """Get ground vehicle name by ID"""
        try:
            return GroundVehicle.objects.get(pk=pk).name
        except GroundVehicle.DoesNotExist:
            return None
