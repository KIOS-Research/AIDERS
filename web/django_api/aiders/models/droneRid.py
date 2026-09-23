from django.contrib.gis.db import models
from django.utils import timezone

from .operation import Operation


class DroneRid(models.Model):
    """Model for tracking drones with remote ID discovered via BLE scanning"""
    
    # Device identification
    address = models.CharField(max_length=17, unique=True, db_index=True, help_text="BLE MAC address")
    name = models.CharField(max_length=255, blank=True, default="Unknown", help_text="Device name")
    basic_id = models.CharField(max_length=255, blank=True, default="", help_text="Basic ID / Serial Number")
    operator_id = models.CharField(max_length=255, blank=True, default="", help_text="Operator ID")
    
    # Service identification
    service_uuids = models.JSONField(default=list, help_text="List of service UUIDs")
    
    # Tracking timestamps
    first_seen = models.DateTimeField(auto_now_add=True, help_text="First time device was detected")
    last_seen = models.DateTimeField(auto_now=True, help_text="Last time device was updated")
    
    # Latest location data (from most recent message)
    location = models.PointField(null=True, blank=True, help_text="Latest GPS coordinates (longitude, latitude)")
    altitude_m = models.FloatField(null=True, blank=True, help_text="Latest altitude in meters")
    speed_m_s = models.FloatField(null=True, blank=True, help_text="Latest speed in m/s")
    
    # Latest signal strength
    rssi = models.IntegerField(null=True, blank=True, help_text="Latest RSSI in dBm")
    
    # Message statistics
    message_counts = models.JSONField(
        default=dict,
        help_text="Message counts: basic_id, operator_id, location, total"
    )

    
    class Meta:
        ordering = ['-last_seen']
        verbose_name = 'Drone Remote ID'
        verbose_name_plural = 'Drone Remote IDs'
        indexes = [
            models.Index(fields=['-last_seen']),
            models.Index(fields=['address']),
        ]
    
    def __str__(self):
        return f"DroneRID {self.address} - {self.basic_id or self.name}"


class DroneRidMessage(models.Model):
    """Model for storing individual drone RID messages for historical tracking"""
    
    # Foreign key to the drone
    drone_rid = models.ForeignKey(
        DroneRid,
        on_delete=models.CASCADE,
        related_name='message_history',
        help_text="The drone this message belongs to"
    )
    
    # Message metadata
    timestamp = models.DateTimeField(db_index=True, help_text="When the message was received")
    rssi = models.IntegerField(null=True, blank=True, help_text="Signal strength in dBm")
    message_type = models.IntegerField(null=True, blank=True, help_text="Message type code")
    subtype = models.IntegerField(null=True, blank=True, help_text="Message subtype code")
    raw_hex = models.CharField(max_length=512, blank=True, help_text="Raw message in hex format")
    
    # Location data
    location = models.PointField(null=True, blank=True, help_text="GPS coordinates (longitude, latitude)")
    latitude = models.FloatField(null=True, blank=True, help_text="Latitude")
    longitude = models.FloatField(null=True, blank=True, help_text="Longitude")
    altitude_m = models.FloatField(null=True, blank=True, help_text="Altitude in meters")
    speed_m_s = models.FloatField(null=True, blank=True, help_text="Speed in m/s")
    
    class Meta:
        db_table = 'aiders_droneridmessage'
        ordering = ['-timestamp']
        verbose_name = 'Drone RID Message'
        verbose_name_plural = 'Drone RID Messages'
        indexes = [
            models.Index(fields=['-timestamp']),
            models.Index(fields=['drone_rid', '-timestamp']),
        ]
    
    def __str__(self):
        return f"Message from {self.drone_rid.address} at {self.timestamp}"
    
    def update_from_scan_data(self, scan_data):
        """
        Update model from scan data structure returned by drid/app/main.py
        
        Args:
            scan_data (dict): Dictionary with keys:
                - address, name, basic_id, operator_id
                - service_uuids, messages, message_counts
                - first_seen (ISO format string)
        """
        # Update identification
        self.name = scan_data.get('name', 'Unknown')
        if scan_data.get('basic_id'):
            self.basic_id = scan_data['basic_id']
        if scan_data.get('operator_id'):
            self.operator_id = scan_data['operator_id']
        
        # Update service UUIDs
        if scan_data.get('service_uuids'):
            self.service_uuids = scan_data['service_uuids']
        
        # Update message counts
        if scan_data.get('message_counts'):
            self.message_counts = scan_data['message_counts']
        
        # Update messages array
        if scan_data.get('messages'):
            # Append new messages to existing ones
            existing_messages = self.messages or []
            new_messages = scan_data['messages']
            self.messages = existing_messages + new_messages
            
            # Update latest location from most recent message
            if new_messages:
                latest_msg = new_messages[-1]
                
                if 'latitude' in latest_msg and 'longitude' in latest_msg:
                    from django.contrib.gis.geos import Point
                    self.location = Point(
                        latest_msg['longitude'],
                        latest_msg['latitude'],
                        srid=4326
                    )
                
                if 'altitude_m' in latest_msg:
                    self.altitude_m = latest_msg['altitude_m']
                
                if 'speed_m_s' in latest_msg:
                    self.speed_m_s = latest_msg['speed_m_s']
                
                if 'rssi' in latest_msg:
                    self.rssi = latest_msg['rssi']


