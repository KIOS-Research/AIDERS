from django.contrib.gis.db import models
from django.utils import timezone


class PilotNotification(models.Model):
    """
    Model to store pilot notifications for communication between pilots and the platform.
    """
    sender = models.CharField(max_length=255)
    receiver = models.CharField(max_length=255, null=True, blank=True)
    incoming = models.BooleanField(default=True, help_text="True if this is an incoming notification, False if outgoing")
    message = models.TextField(help_text="The notification message content")
    timestamp = models.DateTimeField(default=timezone.now, help_text="When the notification was created")
    operation = models.ForeignKey("Operation", on_delete=models.CASCADE)
    
    class Meta:
        ordering = ['-timestamp']  # Most recent first
        verbose_name = "Pilot Notification"
        verbose_name_plural = "Pilot Notifications"


class DeviceNotification(models.Model):
    """
    Model to store device notifications for communication between devices and the platform.
    """
    sender = models.CharField(max_length=255)
    receiver = models.CharField(max_length=255, null=True, blank=True)
    incoming = models.BooleanField(default=True, help_text="True if this is an incoming notification, False if outgoing")
    message = models.TextField(help_text="The notification message content")
    timestamp = models.DateTimeField(default=timezone.now, help_text="When the notification was created")
    operation = models.ForeignKey("Operation", on_delete=models.CASCADE)
    
    class Meta:
        ordering = ['-timestamp']  # Most recent first
        verbose_name = "Device Notification"
        verbose_name_plural = "Device Notifications"
