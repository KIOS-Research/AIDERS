from django.contrib.gis.db import models
from django.core.validators import MinValueValidator
from django.db.models import F, Q
from django.forms import ValidationError
from django.utils import timezone


class FlyingReport(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    user = models.ForeignKey("User", on_delete=models.CASCADE)
    operation = models.ForeignKey("Operation", on_delete=models.CASCADE)
    drone = models.ForeignKey("Drone", on_delete=models.CASCADE, blank=True, null=True)
    latitude = models.FloatField()
    longitude = models.FloatField()
    altitude = models.FloatField(
        validators=[
            MinValueValidator(1.0),
        ]
    )
    radius = models.FloatField(
        validators=[
            MinValueValidator(1.0),
        ]
    )
    buffer_altitude = models.FloatField(
        validators=[
            MinValueValidator(1.0),
        ]
    )
    buffer_radius = models.FloatField(
        validators=[
            MinValueValidator(1.0),
        ]
    )
    start_date_time = models.DateTimeField()
    end_date_time = models.DateTimeField()
    file_path = models.CharField(max_length=255, null=False)

    def clean(self):
        now = timezone.now()
        if self.altitude < 0:
            raise ValidationError("The altitude should not be bellow 0.")
        if self.radius < 0:
            raise ValidationError("The radius should not be bellow 0.")
        if self.buffer_altitude < 0:
            raise ValidationError(
                "The buffer altitude should not be bellow 0.")
        if self.buffer_radius < 0:
            raise ValidationError("The buffer radius should not be bellow 0.")
        if self.start_date_time.date() < now.date():
            raise ValidationError(
                "Start date & time of the operation should not be in the past."
            )
        if self.start_date_time > self.end_date_time:
            raise ValidationError(
                "Start date & time of the operation should be before the end date & time."
            )
        return super().clean()

    class Meta:
        constraints = [
            models.CheckConstraint(
                check=Q(start_date_time__lte=F("end_date_time")),
                name="start_before_end",
            )
        ]
