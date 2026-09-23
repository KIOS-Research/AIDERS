import json
import os
from datetime import datetime
import pytz

timezone = pytz.utc # timezone = pytz.timezone(os.environ.get("TZ"))

# custom libs
from database.connection import MySQLConnector


def getIdByOperatorId(_operatorId):
    query = f"SELECT id FROM aiders_dronerid WHERE operator_id = %s LIMIT 1"
    params = (_operatorId, )
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result[0]


def getDroneRidByAddress(_address):
    """Get drone RID record by BLE MAC address"""
    query = "SELECT id, address, basic_id, operator_id FROM aiders_dronerid WHERE address = %s LIMIT 1"
    params = (_address, )
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result if result else None


def saveDroneRid(_scan_data):
    """
    Insert a new drone RID record from scan data.
    
    Args:
        _scan_data (dict): Scan data with address, name, basic_id, operator_id, 
                          service_uuids, messages, message_counts
    
    Returns:
        int: The ID of the inserted record
    """
    # Extract location from latest message if available
    location_point = None
    altitude_m = None
    speed_m_s = None
    rssi = None
    
    messages = _scan_data.get('messages', [])
    if messages:
        latest_msg = messages[-1]
        if 'latitude' in latest_msg and 'longitude' in latest_msg:
            # Create POINT geometry for PostGIS
            location_point = f"POINT({latest_msg['longitude']} {latest_msg['latitude']})"
        altitude_m = latest_msg.get('altitude_m')
        speed_m_s = latest_msg.get('speed_m_s')
        rssi = latest_msg.get('rssi')
    
    # Build query based on whether we have location data
    if location_point:
        query = (
            "INSERT INTO aiders_dronerid "
            "(address, name, basic_id, operator_id, service_uuids, first_seen, last_seen, "
            "location, altitude_m, speed_m_s, rssi, message_counts, messages) "
            "VALUES (%s, %s, %s, %s, %s, %s, %s, ST_GeomFromText(%s, 4326), %s, %s, %s, %s, %s)"
        )
        params = (
            _scan_data['address'],
            _scan_data.get('name', 'Unknown'),
            _scan_data.get('basic_id', ''),
            _scan_data.get('operator_id', ''),
            json.dumps(_scan_data.get('service_uuids', [])),
            datetime.now(timezone),
            datetime.now(timezone),
            location_point,
            altitude_m,
            speed_m_s,
            rssi,
            json.dumps(_scan_data.get('message_counts', {})),
            json.dumps(_scan_data.get('messages', []))
        )
    else:
        query = (
            "INSERT INTO aiders_dronerid "
            "(address, name, basic_id, operator_id, service_uuids, first_seen, last_seen, "
            "location, altitude_m, speed_m_s, rssi, message_counts, messages) "
            "VALUES (%s, %s, %s, %s, %s, %s, %s, NULL, %s, %s, %s, %s, %s)"
        )
        params = (
            _scan_data['address'],
            _scan_data.get('name', 'Unknown'),
            _scan_data.get('basic_id', ''),
            _scan_data.get('operator_id', ''),
            json.dumps(_scan_data.get('service_uuids', [])),
            datetime.now(timezone),
            datetime.now(timezone),
            altitude_m,
            speed_m_s,
            rssi,
            json.dumps(_scan_data.get('message_counts', {})),
            json.dumps(_scan_data.get('messages', []))
        )
    
    connector = MySQLConnector()
    record_id = connector.executeQuery(query, params, False)
    connector.close()
    return record_id


def updateDroneRid(_id, _scan_data):
    """
    Update existing drone RID record with new scan data.
    
    Args:
        _id (int): The record ID to update
        _scan_data (dict): New scan data to append
    """
    # Get existing record to append messages
    query_select = "SELECT messages, message_counts, basic_id, operator_id FROM aiders_dronerid WHERE id = %s"
    connector = MySQLConnector()
    existing = connector.executeQuery(query_select, (_id,), True)
    
    # Merge messages and counts
    existing_messages = json.loads(existing[0]) if existing and len(existing) > 0 and existing[0] else []
    existing_counts = json.loads(existing[1]) if existing and len(existing) > 1 and existing[1] else {}
    existing_basic_id = existing[2] if existing and len(existing) > 2 else ''
    existing_operator_id = existing[3] if existing and len(existing) > 3 else ''
    
    new_messages = _scan_data.get('messages', [])
    new_counts = _scan_data.get('message_counts', {})
    
    # Append new messages
    combined_messages = existing_messages + new_messages
    
    # Update counts
    combined_counts = existing_counts.copy()
    for key, value in new_counts.items():
        combined_counts[key] = combined_counts.get(key, 0) + value
    
    # Extract location from latest message
    location_point = None
    altitude_m = None
    speed_m_s = None
    rssi = None
    
    if new_messages:
        latest_msg = new_messages[-1]
        if 'latitude' in latest_msg and 'longitude' in latest_msg:
            location_point = f"POINT({latest_msg['longitude']} {latest_msg['latitude']})"
        altitude_m = latest_msg.get('altitude_m')
        speed_m_s = latest_msg.get('speed_m_s')
        rssi = latest_msg.get('rssi')
    
    # Update query - handle location NULL case
    if location_point:
        query_update = (
            "UPDATE aiders_dronerid "
            "SET name = %s, basic_id = %s, operator_id = %s, service_uuids = %s, "
            "last_seen = %s, location = ST_GeomFromText(%s, 4326), altitude_m = %s, "
            "speed_m_s = %s, rssi = %s, message_counts = %s, messages = %s "
            "WHERE id = %s"
        )
        params = (
            _scan_data.get('name', 'Unknown'),
            _scan_data.get('basic_id', '') or existing_basic_id,
            _scan_data.get('operator_id', '') or existing_operator_id,
            json.dumps(_scan_data.get('service_uuids', [])),
            datetime.now(timezone),
            location_point,
            altitude_m,
            speed_m_s,
            rssi,
            json.dumps(combined_counts),
            json.dumps(combined_messages),
            _id
        )
    else:
        query_update = (
            "UPDATE aiders_dronerid "
            "SET name = %s, basic_id = %s, operator_id = %s, service_uuids = %s, "
            "last_seen = %s, altitude_m = %s, "
            "speed_m_s = %s, rssi = %s, message_counts = %s, messages = %s "
            "WHERE id = %s"
        )
        params = (
            _scan_data.get('name', 'Unknown'),
            _scan_data.get('basic_id', '') or existing_basic_id,
            _scan_data.get('operator_id', '') or existing_operator_id,
            json.dumps(_scan_data.get('service_uuids', [])),
            datetime.now(timezone),
            altitude_m,
            speed_m_s,
            rssi,
            json.dumps(combined_counts),
            json.dumps(combined_messages),
            _id
        )
    
    connector.executeQuery(query_update, params, False)
    connector.close()


def saveOrUpdateDroneRid(_scan_data):
    """
    Insert new drone RID record or update existing one based on address.
    
    Args:
        _scan_data (dict): Scan data from BLE scanner
    
    Returns:
        int: The ID of the inserted/updated record
    """
    address = _scan_data.get('address')
    if not address:
        raise ValueError("Scan data must include 'address' field")
    
    # Check if record exists
    existing = getDroneRidByAddress(address)
    
    if existing and len(existing) > 0:
        # Update existing record
        record_id = existing[0]
        updateDroneRid(record_id, _scan_data)
        return record_id
    else:
        # Insert new record
        return saveDroneRid(_scan_data)


def getOrCreateDroneRid(_address, _name="Unknown", _basic_id="", _operator_id="", _service_uuids=None):
    """
    Get existing drone RID record by address or create a new one.
    Used for real-time message saving.
    
    Args:
        _address (str): BLE MAC address
        _name (str): Device name
        _basic_id (str): Basic ID / Serial Number
        _operator_id (str): Operator ID
        _service_uuids (list): List of service UUIDs
    
    Returns:
        int: The ID of the record
    """
    existing = getDroneRidByAddress(_address)
    
    if existing and len(existing) > 0:
        return existing[0]
    
    # Create new record with minimal data
    query = (
        "INSERT INTO aiders_dronerid "
        "(address, name, basic_id, operator_id, service_uuids, first_seen, last_seen, message_counts) "
        "VALUES (%s, %s, %s, %s, %s, %s, %s, %s)"
    )
    
    params = (
        _address,
        _name,
        _basic_id,
        _operator_id,
        json.dumps(_service_uuids or []),
        datetime.now(timezone),
        datetime.now(timezone),
        json.dumps({})
    )
    
    connector = MySQLConnector()
    record_id = connector.executeQuery(query, params, False)
    connector.close()
    return record_id


def saveDroneRidMessage(_drone_rid_id, _message_data):
    """
    Save an individual drone RID message for historical tracking.
    
    Args:
        _drone_rid_id (int): The drone RID record ID
        _message_data (dict): Message data with timestamp, location, rssi, etc.
    
    Returns:
        int: The ID of the inserted message record
    """
    # Extract location
    location_point = None
    latitude = _message_data.get('latitude')
    longitude = _message_data.get('longitude')
    
    if latitude is not None and longitude is not None:
        location_point = f"POINT({longitude} {latitude})"
    
    # Parse timestamp - use current time if not provided
    timestamp_str = _message_data.get('timestamp')
    if timestamp_str:
        # Parse ISO format timestamp using standard library
        try:
            msg_timestamp = datetime.fromisoformat(timestamp_str.replace('Z', '+00:00'))
        except:
            msg_timestamp = datetime.now(timezone)
    else:
        msg_timestamp = datetime.now(timezone)
    
    # Build query based on whether we have location
    if location_point:
        query = (
            "INSERT INTO aiders_droneridmessage "
            "(drone_rid_id, timestamp, rssi, message_type, subtype, raw_hex, "
            "location, latitude, longitude, altitude_m, speed_m_s) "
            "VALUES (%s, %s, %s, %s, %s, %s, ST_GeomFromText(%s, 4326), %s, %s, %s, %s)"
        )
        params = (
            _drone_rid_id,
            msg_timestamp,
            _message_data.get('rssi'),
            _message_data.get('message_type'),
            _message_data.get('subtype'),
            _message_data.get('raw_hex', ''),
            location_point,
            latitude,
            longitude,
            _message_data.get('altitude_m'),
            _message_data.get('speed_m_s')
        )
    else:
        query = (
            "INSERT INTO aiders_droneridmessage "
            "(drone_rid_id, timestamp, rssi, message_type, subtype, raw_hex, "
            "latitude, longitude, altitude_m, speed_m_s) "
            "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
        )
        params = (
            _drone_rid_id,
            msg_timestamp,
            _message_data.get('rssi'),
            _message_data.get('message_type'),
            _message_data.get('subtype'),
            _message_data.get('raw_hex', ''),
            latitude,
            longitude,
            _message_data.get('altitude_m'),
            _message_data.get('speed_m_s')
        )
    
    connector = MySQLConnector()
    message_id = connector.executeQuery(query, params, False)
    connector.close()
    return message_id


def updateDroneRidIdentity(_drone_rid_id, _basic_id=None, _operator_id=None, _name=None):
    """
    Update the identity fields (basic_id, operator_id, name) for a drone RID record.
    Only updates non-None values.
    
    Args:
        _drone_rid_id (int): The drone RID record ID
        _basic_id (str, optional): Basic ID / Serial Number
        _operator_id (str, optional): Operator ID
        _name (str, optional): Device name
    """
    # Build dynamic update query based on provided fields
    update_fields = []
    params = []
    
    if _basic_id is not None and _basic_id != "":
        update_fields.append("basic_id = %s")
        params.append(_basic_id)
    
    if _operator_id is not None and _operator_id != "":
        update_fields.append("operator_id = %s")
        params.append(_operator_id)
    
    if _name is not None and _name != "":
        update_fields.append("name = %s")
        params.append(_name)
    
    # Always update last_seen
    update_fields.append("last_seen = %s")
    params.append(datetime.now(timezone))
    
    # Add the ID parameter
    params.append(_drone_rid_id)
    
    if len(update_fields) > 1:  # More than just last_seen
        query = f"UPDATE aiders_dronerid SET {', '.join(update_fields)} WHERE id = %s"
        connector = MySQLConnector()
        connector.executeQuery(query, tuple(params), False)
        connector.close()


def updateDroneRidLatestData(_drone_rid_id, _message_data):
    """
    Update the latest location and message count for a drone RID record.
    
    Args:
        _drone_rid_id (int): The drone RID record ID
        _message_data (dict): Latest message data
    """
    # Extract location
    location_point = None
    latitude = _message_data.get('latitude')
    longitude = _message_data.get('longitude')
    
    if latitude is not None and longitude is not None:
        location_point = f"POINT({longitude} {latitude})"
    
    connector = MySQLConnector()
    
    # Update query
    if location_point:
        query_update = (
            "UPDATE aiders_dronerid "
            "SET last_seen = %s, location = ST_GeomFromText(%s, 4326), altitude_m = %s, "
            "speed_m_s = %s, rssi = %s "
            "WHERE id = %s"
        )
        params = (
            datetime.now(timezone),
            location_point,
            _message_data.get('altitude_m'),
            _message_data.get('speed_m_s'),
            _message_data.get('rssi'),
            _drone_rid_id
        )
    else:
        query_update = (
            "UPDATE aiders_dronerid "
            "SET last_seen = %s, rssi = %s "
            "WHERE id = %s"
        )
        params = (
            datetime.now(timezone),
            _message_data.get('rssi'),
            _drone_rid_id
        )
    
    connector.executeQuery(query_update, params, False)
    connector.close()

