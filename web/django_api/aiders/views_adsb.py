# ADS-B Aircraft Data API
# Provides aircraft tracking data for the AIDERS platform

import json
import logging
import requests
from datetime import datetime, timedelta
from django.http import JsonResponse
from django.views.decorators.http import require_http_methods
from django.views.decorators.csrf import csrf_exempt
from django.core.cache import cache
from .models import AdsbAircraft, AdsbAircraftHistory

logger = logging.getLogger(__name__)

# OpenSky Network API Configuration

@require_http_methods(["GET"])
def get_aircraft_trail(request, operation_name, icao24):
    """
    Get historical trail data for a specific aircraft.
    
    Args:
        request: HTTP request object
        operation_name: Name of the operation (from URL path)
        icao24: ICAO 24-bit address of the aircraft
    
    Query Parameters:
    - since: ISO format datetime string (optional, defaults to last 30 minutes)
    - limit: Maximum number of points to return (optional, defaults to 1000)
    
    Returns:
        JSON array of trail points with timestamp, latitude, longitude, altitude, speed, track
    """
    try:
        # Get aircraft by ICAO
        aircraft = AdsbAircraft.objects.filter(icao24=icao24).first()
        if not aircraft:
            return JsonResponse({"error": "Aircraft not found"}, status=404)
        
        # Parse query parameters
        since_str = request.GET.get('since')
        limit = int(request.GET.get('limit', 1000))
        
        # Default to last 30 minutes if not specified
        if since_str:
            try:
                since = datetime.fromisoformat(since_str.replace('Z', '+00:00'))
            except ValueError:
                since = datetime.now() - timedelta(minutes=30)
        else:
            since = datetime.now() - timedelta(minutes=30)
        
        # Query trail history
        trail_query = AdsbAircraftHistory.objects.filter(
            aircraft=aircraft,
            timestamp__gte=since
        ).order_by('timestamp')[:limit]
        
        # Format trail data
        trail = []
        for point in trail_query:
            trail.append({
                'timestamp': point.timestamp.isoformat(),
                'latitude': point.latitude,
                'longitude': point.longitude,
                'altitude_ft': point.altitude_ft,
                'ground_speed_kts': point.ground_speed_kts,
                'track_deg': point.track_deg,
                'vertical_rate_fpm': point.vertical_rate_fpm
            })
        
        return JsonResponse({
            'icao24': icao24,
            'trail': trail,
            'count': len(trail)
        })
        
    except Exception as e:
        logger.error(f"Error retrieving aircraft trail for {icao24}: {e}")
        return JsonResponse({"error": str(e)}, status=500)
