import datetime
import json
import os

from django.contrib.gis.db import models
from django.core import serializers as coreDjangoSerializers
from django.db import transaction
from django.utils import timezone

from logic.Constants import Constants
from .drone import Drone


class SafeDroneResults(models.Model):
    drone = models.ForeignKey("Drone", on_delete=models.CASCADE)
    motorPfail = models.DecimalField(max_digits=8, decimal_places=2)
    motorMTTF = models.DecimalField(max_digits=8, decimal_places=2)
    batteryPfail = models.DecimalField(max_digits=8, decimal_places=2)
    batteryMTTF = models.DecimalField(max_digits=8, decimal_places=2)
    chipPfail = models.DecimalField(max_digits=8, decimal_places=2)
    chipMTTF = models.DecimalField(max_digits=8, decimal_places=2)
    gpsPfail = models.DecimalField(max_digits=8, decimal_places=2, null=True)
    gpsMTTF = models.DecimalField(max_digits=8, decimal_places=2, null=True)    
    Seconds = models.IntegerField()
    DateTime = models.DateTimeField(auto_now_add=True)


