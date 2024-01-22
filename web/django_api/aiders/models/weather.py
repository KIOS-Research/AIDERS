
import datetime
import json

from django.contrib.gis.db import models
from django.core import serializers as coreDjangoSerializers
from django.utils import timezone

from .drone import Drone


class WeatherStation(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    wind_speed = models.FloatField()
    wind_direction = models.FloatField()
    temperature = models.FloatField()
    pressure = models.FloatField()
    humidity = models.FloatField()
    heading = models.FloatField()
    operation = models.ForeignKey(
        "Operation", on_delete=models.CASCADE, blank=True, null=True)
    drone = models.ForeignKey(
        "Drone", on_delete=models.CASCADE, blank=True, null=True)

    def getAllWeatherDataOfOperationBetweenTwoTimes(operationName, _startTime, _endTime):
        return WeatherStation.objects.filter(
            drone__isnull=True,
            time__gte=_startTime,
            time__lte=_endTime,
        )

    def getAllWeatherDataOfDroneBetweenTwoTimes(_droneName, _startTime, _endTime):
        return WeatherStation.objects.filter(
            drone__drone_name=_droneName,
            time__gte=_startTime,
            time__lte=_endTime,
        )

    def convertListWeatherDataToJsonFormat(listOfWeatherData):
        listOfWeatherDataJson = json.loads(
            coreDjangoSerializers.serialize("json", listOfWeatherData))
        for recordOfWeatherDataJson in listOfWeatherDataJson:
            fields = recordOfWeatherDataJson["fields"]
            if "time" in fields:
                fields["time"] = timezone.make_aware(datetime.datetime.fromisoformat(
                    fields["time"][:-1]), timezone.utc).timestamp()
            fields["operation"] = ""
            if fields["drone"] == None:
                fields["drone"] = ""
                fields["type"] = "weatherStation"
            else:
                fields["drone"] = Drone.getDroneNameById(fields["drone"])
                fields["type"] = "weatherDrone"
        return [recordOfWeatherDataJson["fields"] for recordOfWeatherDataJson in listOfWeatherDataJson]
