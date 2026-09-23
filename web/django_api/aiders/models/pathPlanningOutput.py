from django.db import models


class PathPlanningOutput(models.Model):
    message = models.JSONField(default=list)
    processed = models.BooleanField(null=True, default=False)
    time = models.DateTimeField(null=True, blank=True)
