from django.contrib.gis.db import models
from django.utils import timezone


class AdsbAircraft(models.Model):
    """Model for tracking aircraft detected via ADS-B signals"""
    
    # Aircraft identification
    icao24 = models.CharField(
        max_length=6, 
        unique=True, 
        db_index=True, 
        help_text="ICAO 24-bit address (hex)"
    )
    callsign = models.CharField(
        max_length=8, 
        blank=True, 
        default="", 
        help_text="Aircraft callsign/flight number"
    )
    
    # Tracking timestamps
    first_seen = models.DateTimeField(
        auto_now_add=True, 
        help_text="First time aircraft was detected"
    )
    last_seen = models.DateTimeField(
        auto_now=True, 
        help_text="Last time aircraft was updated"
    )
    
    # Position data
    location = models.PointField(
        null=True, 
        blank=True, 
        help_text="Latest GPS coordinates (longitude, latitude)"
    )
    latitude = models.FloatField(
        null=True, 
        blank=True, 
        help_text="Latitude in decimal degrees"
    )
    longitude = models.FloatField(
        null=True, 
        blank=True, 
        help_text="Longitude in decimal degrees"
    )
    altitude_ft = models.IntegerField(
        null=True, 
        blank=True, 
        help_text="Altitude in feet"
    )
    
    # Movement data
    ground_speed_kts = models.IntegerField(
        null=True, 
        blank=True, 
        help_text="Ground speed in knots"
    )
    track_deg = models.FloatField(
        null=True, 
        blank=True, 
        help_text="Track/heading in degrees"
    )
    vertical_rate_fpm = models.IntegerField(
        null=True, 
        blank=True, 
        help_text="Vertical rate in feet per minute"
    )
    
    # Additional data
    squawk = models.CharField(
        max_length=4, 
        blank=True, 
        default="", 
        help_text="Squawk code"
    )
    emergency = models.BooleanField(
        default=False, 
        help_text="Emergency flag"
    )
    on_ground = models.BooleanField(
        default=False, 
        help_text="On ground flag"
    )
    
    class Meta:
        db_table = 'aiders_adsbaircraft'
        ordering = ['-last_seen']
        verbose_name = 'ADS-B Aircraft'
        verbose_name_plural = 'ADS-B Aircraft'
        indexes = [
            models.Index(fields=['-last_seen']),
            models.Index(fields=['icao24']),
            models.Index(fields=['callsign']),
        ]
    
    def __str__(self):
        if self.callsign:
            return f"Aircraft {self.icao24} ({self.callsign})"
        return f"Aircraft {self.icao24}"
    
    def to_dict(self):
        """Convert model to dictionary for API responses"""
        return {
            'id': self.id,
            'icao24': self.icao24,
            'callsign': self.callsign,
            'first_seen': self.first_seen.isoformat() if self.first_seen else None,
            'last_seen': self.last_seen.isoformat() if self.last_seen else None,
            'latitude': self.latitude,
            'longitude': self.longitude,
            'altitude_ft': self.altitude_ft,
            'ground_speed_kts': self.ground_speed_kts,
            'track_deg': self.track_deg,
            'vertical_rate_fpm': self.vertical_rate_fpm,
            'squawk': self.squawk,
            'emergency': self.emergency,
            'on_ground': self.on_ground,
        }


class AdsbAircraftHistory(models.Model):
    """Model for storing historical position data for ADS-B aircraft"""
    
    # Foreign key to the aircraft
    aircraft = models.ForeignKey(
        AdsbAircraft,
        on_delete=models.CASCADE,
        related_name='history',
        help_text="The aircraft this position belongs to"
    )
    
    # Timestamp
    timestamp = models.DateTimeField(
        db_index=True, 
        help_text="When the position was recorded"
    )
    
    # Position data
    location = models.PointField(
        null=True, 
        blank=True, 
        help_text="GPS coordinates (longitude, latitude)"
    )
    latitude = models.FloatField(
        null=True, 
        blank=True, 
        help_text="Latitude in decimal degrees"
    )
    longitude = models.FloatField(
        null=True, 
        blank=True, 
        help_text="Longitude in decimal degrees"
    )
    altitude_ft = models.IntegerField(
        null=True, 
        blank=True, 
        help_text="Altitude in feet"
    )
    
    # Movement data
    ground_speed_kts = models.IntegerField(
        null=True, 
        blank=True, 
        help_text="Ground speed in knots"
    )
    track_deg = models.FloatField(
        null=True, 
        blank=True, 
        help_text="Track/heading in degrees"
    )
    vertical_rate_fpm = models.IntegerField(
        null=True, 
        blank=True, 
        help_text="Vertical rate in feet per minute"
    )
    
    class Meta:
        db_table = 'aiders_adsbaircraft_history'
        ordering = ['-timestamp']
        verbose_name = 'ADS-B Aircraft History'
        verbose_name_plural = 'ADS-B Aircraft History'
        indexes = [
            models.Index(fields=['-timestamp']),
            models.Index(fields=['aircraft', '-timestamp']),
        ]
    
    def __str__(self):
        return f"Position for {self.aircraft.icao24} at {self.timestamp}"
