
import datetime
import json

from django.contrib.gis.db import models
from django.core import serializers as coreDjangoSerializers
from django.utils import timezone

from .drone import Drone


class WeatherAPI(models.Model):
    """Model to store weather data from WeatherAPI.com"""
    city = models.CharField(max_length=100)
    country = models.CharField(max_length=100, blank=True, null=True)  # Country name
    region = models.CharField(max_length=100, blank=True, null=True)  # Region/State
    latitude = models.FloatField(blank=True, null=True)  # Latitude for map display
    longitude = models.FloatField(blank=True, null=True)  # Longitude for map display
    temperature = models.FloatField()  # in Celsius
    feels_like = models.FloatField()  # in Celsius
    humidity = models.IntegerField()  # percentage
    pressure = models.FloatField()  # mb (millibars)
    wind_speed = models.FloatField()  # kph
    wind_direction = models.FloatField()  # degrees
    wind_dir_text = models.CharField(max_length=10, blank=True, null=True)  # Wind direction text (N, NE, etc.)
    weather_condition = models.CharField(max_length=100)  # weather condition text
    weather_icon = models.CharField(max_length=50, blank=True, null=True)  # weather icon code
    visibility = models.FloatField(blank=True, null=True)  # km
    uv_index = models.FloatField(blank=True, null=True)
    cloud_cover = models.IntegerField(blank=True, null=True)  # percentage
    gust_kph = models.FloatField(blank=True, null=True)  # wind gust in kph
    timestamp = models.DateTimeField(auto_now_add=True)
    api_timestamp = models.DateTimeField()  # timestamp from API
    
    class Meta:
        ordering = ['-timestamp']
        
    def __str__(self):
        return f"{self.city} - {self.weather_condition} ({self.timestamp})"


class WeatherConfig(models.Model):
    """Model to store WeatherAPI.com API configuration"""
    api_key = models.CharField(max_length=100)
    cities = models.TextField(help_text="Comma-separated list of cities")
    update_interval = models.IntegerField(default=60, help_text="Update interval in minutes")
    is_active = models.BooleanField(default=True)
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)
    
    def get_cities_list(self):
        """Return list of cities from comma-separated string"""
        return [city.strip() for city in self.cities.split(',') if city.strip()]
    
    def __str__(self):
        return f"Weather Config - {len(self.get_cities_list())} cities"


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
