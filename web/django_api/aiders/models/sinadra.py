from django.contrib.gis.db import models
from .operation import Operation

class SinadraData(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    operation = models.ForeignKey(Operation, on_delete=models.CASCADE)
    human_injury_risk_prediction = models.FloatField()