from django.contrib.auth import get_user_model
from django.contrib.gis.db import models


class ManuallySetObject(models.Model):
    created_by = models.ForeignKey(get_user_model(), on_delete=models.DO_NOTHING)
    created_at = models.DateTimeField(auto_now_add=True)
    operation = models.ForeignKey("Operation", on_delete=models.CASCADE)
    description = models.CharField(max_length=300, null=True, blank=True)
    label = models.CharField(max_length=100, null=True)


class ManuallySetObjectLocation(models.Model):
    received_by = models.ForeignKey(get_user_model(), on_delete=models.DO_NOTHING)
    received_at = models.DateTimeField(auto_now_add=True)
    lat = models.FloatField()
    lon = models.FloatField()
    manually_set_object = models.ForeignKey(ManuallySetObject, on_delete=models.CASCADE)


class ManuallySetObjectDescription(models.Model):
    manually_set_object = models.ForeignKey(ManuallySetObject, on_delete=models.CASCADE)
    description = models.CharField(max_length=300, null=True, blank=True)
    updated_at = models.DateTimeField(auto_now_add=True)
    updated_by = models.ForeignKey(get_user_model(), on_delete=models.DO_NOTHING)
