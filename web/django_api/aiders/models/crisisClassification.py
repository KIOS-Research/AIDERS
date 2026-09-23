from django.db import models
from django.contrib.auth import get_user_model
class CrisisClassification(models.Model):
    # Header fields
    sent_utc = models.DateTimeField()
    sender = models.CharField(max_length=100)
    # action_type = models.CharField(max_length=50)

    # Body fields
    incident_id = models.CharField(max_length=50)
    incident_type = models.CharField(max_length=100)
    severity_level = models.CharField(max_length=50)
    latitude = models.DecimalField(max_digits=10, decimal_places=7)
    longitude = models.DecimalField(max_digits=10, decimal_places=7)


    description = models.TextField(blank=True, null=True)
    resolved = models.BooleanField(default=False, null=True)
    false_alarm = models.BooleanField(default=False, null=True)
    updated_at = models.DateTimeField(auto_now=True, null=True)  # <-- this will auto-update on save
    updated_by = models.ForeignKey(get_user_model(), on_delete=models.DO_NOTHING, null=True)

    timestamp = models.DateTimeField()
    area_type = models.CharField(max_length=20, null=True)
    time_of_day = models.CharField(max_length=20, null=True)
    detections_summary = models.JSONField(blank=True, null=True)

    def save(self, *args, **kwargs):
        super().save(*args, **kwargs)

    def __str__(self) -> str:
        return self.incident_id
    
    def getCrisisByIncidentId(incident_id):
        try:
            return CrisisClassification.objects.get(incident_id=incident_id)
        except CrisisClassification.DoesNotExist:
            return None



