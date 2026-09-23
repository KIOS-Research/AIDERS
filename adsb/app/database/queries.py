import json
import os
from datetime import datetime
import pytz

timezone = pytz.utc

# custom libs
from database.connection import MySQLConnector


def getAircraftByIcao(_icao24):
    """Get aircraft record by ICAO 24-bit address"""
    query = "SELECT id, icao24, callsign FROM aiders_adsbaircraft WHERE icao24 = %s LIMIT 1"
    params = (_icao24, )
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result if result else None


def saveAircraft(_aircraft_data):
    """
    Insert a new aircraft record from ADS-B data.
    
    Args:
        _aircraft_data (dict): Aircraft data with icao24, callsign, latitude, longitude,
                              altitude_ft, ground_speed_kts, track_deg, vertical_rate_fpm,
                              squawk, emergency, on_ground, timestamp
    
    Returns:
        int: The ID of the inserted record
    """
    # Extract location data
    location_point = None
    
    if 'latitude' in _aircraft_data and 'longitude' in _aircraft_data:
        # Create POINT geometry for PostGIS
        location_point = f"POINT({_aircraft_data['longitude']} {_aircraft_data['latitude']})"
    
    # Build query based on whether we have location data
    if location_point:
        query = (
            "INSERT INTO aiders_adsbaircraft "
            "(icao24, callsign, location, latitude, longitude, altitude_ft, "
            "ground_speed_kts, track_deg, vertical_rate_fpm, squawk, emergency, "
            "on_ground, first_seen, last_seen) "
            "VALUES (%s, %s, ST_GeomFromText(%s, 4326), %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
        )
        params = (
            _aircraft_data['icao24'],
            _aircraft_data.get('callsign', ''),
            location_point,
            _aircraft_data.get('latitude'),
            _aircraft_data.get('longitude'),
            _aircraft_data.get('altitude_ft'),
            _aircraft_data.get('ground_speed_kts'),
            _aircraft_data.get('track_deg'),
            _aircraft_data.get('vertical_rate_fpm'),
            _aircraft_data.get('squawk', ''),
            _aircraft_data.get('emergency', False),
            _aircraft_data.get('on_ground', False),
            datetime.now(timezone),
            datetime.now(timezone)
        )
    else:
        query = (
            "INSERT INTO aiders_adsbaircraft "
            "(icao24, callsign, location, latitude, longitude, altitude_ft, "
            "ground_speed_kts, track_deg, vertical_rate_fpm, squawk, emergency, "
            "on_ground, first_seen, last_seen) "
            "VALUES (%s, %s, NULL, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
        )
        params = (
            _aircraft_data['icao24'],
            _aircraft_data.get('callsign', ''),
            _aircraft_data.get('latitude'),
            _aircraft_data.get('longitude'),
            _aircraft_data.get('altitude_ft'),
            _aircraft_data.get('ground_speed_kts'),
            _aircraft_data.get('track_deg'),
            _aircraft_data.get('vertical_rate_fpm'),
            _aircraft_data.get('squawk', ''),
            _aircraft_data.get('emergency', False),
            _aircraft_data.get('on_ground', False),
            datetime.now(timezone),
            datetime.now(timezone)
        )
    
    connector = MySQLConnector()
    record_id = connector.executeQuery(query, params, False)
    connector.close()
    return record_id


def updateAircraft(_id, _aircraft_data):
    """
    Update existing aircraft record with new ADS-B data.
    
    Args:
        _id (int): The record ID to update
        _aircraft_data (dict): New aircraft data
    """
    # Extract location data
    location_point = None
    
    if 'latitude' in _aircraft_data and 'longitude' in _aircraft_data:
        location_point = f"POINT({_aircraft_data['longitude']} {_aircraft_data['latitude']})"
    
    # Build update query
    if location_point:
        query = (
            "UPDATE aiders_adsbaircraft SET "
            "callsign = %s, location = ST_GeomFromText(%s, 4326), latitude = %s, longitude = %s, "
            "altitude_ft = %s, ground_speed_kts = %s, track_deg = %s, vertical_rate_fpm = %s, "
            "squawk = %s, emergency = %s, on_ground = %s, last_seen = %s "
            "WHERE id = %s"
        )
        params = (
            _aircraft_data.get('callsign', ''),
            location_point,
            _aircraft_data.get('latitude'),
            _aircraft_data.get('longitude'),
            _aircraft_data.get('altitude_ft'),
            _aircraft_data.get('ground_speed_kts'),
            _aircraft_data.get('track_deg'),
            _aircraft_data.get('vertical_rate_fpm'),
            _aircraft_data.get('squawk', ''),
            _aircraft_data.get('emergency', False),
            _aircraft_data.get('on_ground', False),
            datetime.now(timezone),
            _id
        )
    else:
        # Update without location change
        query = (
            "UPDATE aiders_adsbaircraft SET "
            "callsign = %s, altitude_ft = %s, ground_speed_kts = %s, track_deg = %s, "
            "vertical_rate_fpm = %s, squawk = %s, emergency = %s, on_ground = %s, last_seen = %s "
            "WHERE id = %s"
        )
        params = (
            _aircraft_data.get('callsign', ''),
            _aircraft_data.get('altitude_ft'),
            _aircraft_data.get('ground_speed_kts'),
            _aircraft_data.get('track_deg'),
            _aircraft_data.get('vertical_rate_fpm'),
            _aircraft_data.get('squawk', ''),
            _aircraft_data.get('emergency', False),
            _aircraft_data.get('on_ground', False),
            datetime.now(timezone),
            _id
        )
    
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()


def saveOrUpdateAircraft(_aircraft_data):
    """
    Save or update aircraft data in the database.
    Checks if aircraft exists by ICAO address and updates if found, otherwise creates new record.
    Also saves historical position data if location is available.
    Only saves data that includes latitude and longitude.
    
    Args:
        _aircraft_data (dict): Aircraft data from ADS-B scanner
    
    Returns:
        int: The ID of the aircraft record, or None if skipped
    """
    icao24 = _aircraft_data.get('icao24')
    
    if not icao24:
        print("Error: No ICAO address provided")
        return None
    
    # Skip if no position data
    if 'latitude' not in _aircraft_data or 'longitude' not in _aircraft_data:
        # Silently skip - position messages come separately from identification messages
        return None
    
    # Check if aircraft already exists
    existing = getAircraftByIcao(icao24)
    
    if existing:
        # Update existing record
        aircraft_id = existing[0]
        updateAircraft(aircraft_id, _aircraft_data)
    else:
        # Create new record
        aircraft_id = saveAircraft(_aircraft_data)
    
    # Save to history table if we have position data
    if aircraft_id and 'latitude' in _aircraft_data and 'longitude' in _aircraft_data:
        saveAircraftHistory(aircraft_id, _aircraft_data)
    
    return aircraft_id


def saveAircraftHistory(_aircraft_id, _aircraft_data):
    """
    Save historical position data for an aircraft.
    
    Args:
        _aircraft_id (int): The aircraft record ID
        _aircraft_data (dict): Aircraft data with position, altitude, speed, etc.
    
    Returns:
        int: The ID of the history record
    """
    # Extract location data
    location_point = None
    
    if 'latitude' in _aircraft_data and 'longitude' in _aircraft_data:
        location_point = f"POINT({_aircraft_data['longitude']} {_aircraft_data['latitude']})"
    
    # Build query
    if location_point:
        query = (
            "INSERT INTO aiders_adsbaircraft_history "
            "(aircraft_id, timestamp, location, latitude, longitude, altitude_ft, "
            "ground_speed_kts, track_deg, vertical_rate_fpm) "
            "VALUES (%s, %s, ST_GeomFromText(%s, 4326), %s, %s, %s, %s, %s, %s)"
        )
        params = (
            _aircraft_id,
            datetime.now(timezone),
            location_point,
            _aircraft_data.get('latitude'),
            _aircraft_data.get('longitude'),
            _aircraft_data.get('altitude_ft'),
            _aircraft_data.get('ground_speed_kts'),
            _aircraft_data.get('track_deg'),
            _aircraft_data.get('vertical_rate_fpm')
        )
    else:
        # No location data, skip history save
        return None
    
    connector = MySQLConnector()
    record_id = connector.executeQuery(query, params, False)
    connector.close()
    return record_id
