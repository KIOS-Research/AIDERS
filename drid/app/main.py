#!/usr/bin/env python3
"""
BLE Scanner for Drone Remote ID Devices
Scans for BLE devices broadcasting the Remote ID service UUID: 0000fffa-0000-1000-8000-00805f9b34fb
"""

import os
import asyncio
import json
from datetime import datetime
from bleak import BleakScanner, BleakClient
from dotenv import load_dotenv

# Custom database imports
import database.connection
from database.queries import getOrCreateDroneRid, saveDroneRidMessage, updateDroneRidLatestData, updateDroneRidIdentity

# Remote ID Service UUID (as per ASTM F3411 / ASD-STAN prEN 4709-002)
REMOTE_ID_SERVICE_UUID = "0000fffa-0000-1000-8000-00805f9b34fb"

# Message type mapping (ASTM F3411 standard)
MESSAGE_TYPES = {
    0x0: "Basic ID",
    0x1: "Location/Vector",
    0x2: "Authentication",
    0x3: "Self-ID",
    0x4: "System",
    0x5: "Operator ID",
    0xF: "Message Pack"
}

# Global flag to enable/disable real-time database saving
SAVE_TO_DB_REALTIME = True
# Cache of drone RID IDs to avoid repeated lookups
drone_rid_cache = {}



def parse_operator_id_message(data_bytes):
    """Parse Operator ID message (type 3)"""
    if len(data_bytes) < 3:
        return None
    
    result = {
        "message_type": 3,
        "message_type_name": "OperatorID",
        "id_type": data_bytes[2],
        "operator_id": ""
    }
    
    # ID types: 0=Text, 1=International, 2=Operator ID
    if data_bytes[2] in [0x00, 0x52]:  # Text format
        # Extract operator ID from bytes 3-22
        id_bytes = data_bytes[3:23]
        # Convert to string, removing null bytes
        result["operator_id"] = bytes(id_bytes).decode('ascii', errors='ignore').rstrip('\x00')
    
    print(f"Parsed Operator ID: {result['operator_id']}")
    return result



def parse_manufacturer_location(data_bytes):
    """Parse location data from manufacturer data"""
    if len(data_bytes) < 23:
        return None
    
    # Byte 1 should be 0x08 for location message
    msg_type = data_bytes[1]
    if msg_type != 0x08:
        return None
    
    result = {
        "message_type": 8,
        "message_type_name": "Location",
        "frame_counter": data_bytes[0]
    }
    
    try:
        # Latitude (bytes 2-5, little-endian int32, multiply by 1e-7)
        lat_raw = int.from_bytes(data_bytes[2:6], byteorder='little', signed=True)
        result["latitude"] = lat_raw * 1e-7
        
        # Longitude (bytes 6-9, little-endian int32, multiply by 1e-7)
        lon_raw = int.from_bytes(data_bytes[6:10], byteorder='little', signed=True)
        result["longitude"] = lon_raw * 1e-7
        
        # Altitude MSL (bytes 10-11, little-endian uint16, multiply by 0.5)
        alt_raw = int.from_bytes(data_bytes[10:12], byteorder='little', signed=False)
        result["altitude_msl"] = alt_raw * 0.5
        
        # Height AGL (byte 12, signed int8, multiply by 0.5)
        height_raw = int.from_bytes([data_bytes[12]], byteorder='little', signed=True)
        result["height_agl"] = height_raw * 0.5
        
        # Accuracy fields (bytes 13-16)
        result["horizontal_accuracy"] = data_bytes[13]
        result["vertical_accuracy"] = data_bytes[14]
        result["barometric_accuracy"] = data_bytes[15]
        result["speed_accuracy"] = data_bytes[16]
        
        # Horizontal speed (bytes 17-18, little-endian uint16, cm/s)
        speed_raw = int.from_bytes(data_bytes[17:19], byteorder='little', signed=False)
        result["horizontal_speed_m_s"] = speed_raw / 100.0
        
        # Vertical speed (byte 19, signed int8, multiply by 0.5 for cm/s)
        vert_speed_raw = int.from_bytes([data_bytes[19]], byteorder='little', signed=True)
        result["vertical_speed_m_s"] = (vert_speed_raw * 0.5) / 100.0
        
        # Direction (byte 20, degrees)
        result["direction_deg"] = data_bytes[20]
        
        # Timestamp (bytes 21-22, little-endian uint16, 0.1s since hour)
        timestamp_raw = int.from_bytes(data_bytes[21:23], byteorder='little', signed=False)
        result["timestamp_deciseconds"] = timestamp_raw
        minutes = (timestamp_raw // 600)
        seconds = (timestamp_raw % 600) / 10.0
        result["time_since_hour"] = f"{minutes}m {seconds:.1f}s"
        
    except Exception as e:
        result["parse_error"] = str(e)
    
    return result




async def scan_for_drone_remote_id_continuous():
    """
    Continuously scan for Remote ID devices indefinitely until interrupted.
    Saves messages to database in real-time.
    
    Returns:
        None
    """
    print(f"Starting continuous scan for Drone Remote ID devices...")
    print(f"Looking for service UUID: {REMOTE_ID_SERVICE_UUID}")
    print(f"Press Ctrl+C to stop scanning\n")
    
    detected_devices = {}
    known_beacon_addresses = set()  # Track MAC addresses of confirmed Remote ID beacons
    
    def detection_callback(device, advertisement_data):
        """Callback for each advertisement received."""
        service_uuids = advertisement_data.service_uuids
        address = device.address
        
        # Check if this is a known beacon OR has Remote ID service UUID
        has_rid_service = REMOTE_ID_SERVICE_UUID in service_uuids
        is_known_beacon = address in known_beacon_addresses
        
        if has_rid_service or is_known_beacon:
            # If it has the service UUID, mark it as a known beacon
            if has_rid_service:
                known_beacon_addresses.add(address)
            
            # Try to get service data
            if REMOTE_ID_SERVICE_UUID in advertisement_data.service_data:
                service_data = advertisement_data.service_data[REMOTE_ID_SERVICE_UUID]

                print(f"\n{address} - Message: {service_data.hex()}")

                if len(service_data) < 3:
                    print("  Message too short to parse.")
                    return

                message_type = service_data[1]
                subtype = service_data[2]  # Byte 2 indicates the subtype
                
                # Message type 2 with subtype 0x02 = Basic ID / Serial Number
                if message_type == 0x02 and subtype == 0x02:
                    # This is a Basic ID message
                    id_bytes = service_data[4:23] if len(service_data) >= 23 else service_data[4:]
                    basic_id = id_bytes.decode('ascii', errors='ignore').rstrip('\x00').strip()
                    print(f"  Type: Basic ID / Serial Number")
                    print(f"  ID: {basic_id}")
                
                # Message type 2 with subtype 0x52 = Authentication/Operator ID
                elif message_type == 0x02 and subtype == 0x52:
                    # This is an Operator ID message
                    # Byte 3 is often 0x00 (null), actual ID starts at byte 4
                    id_bytes = service_data[4:24] if len(service_data) >= 24 else service_data[4:]
                    operator_id = id_bytes.decode('ascii', errors='ignore').rstrip('\x00').strip()
                    print(f"  Type: Operator ID")
                    print(f"  ID: {operator_id}")
                    
                # Subtype 0x42 = Manufacturer location data (can have various message types)
                elif subtype == 0x42:
                    # This is location data with coordinates at bytes 4-11
                    if len(service_data) >= 12:
                        lat_raw = int.from_bytes(service_data[4:8], byteorder='little', signed=True)
                        lon_raw = int.from_bytes(service_data[8:12], byteorder='little', signed=True)
                        latitude = lat_raw * 1e-7
                        longitude = lon_raw * 1e-7
                        
                        print(f"  Type: Location (Manufacturer, 0x42)")
                        print(f"  Latitude: {latitude:.7f}°")
                        print(f"  Longitude: {longitude:.7f}°")
                        
                        # Parse additional telemetry if available
                        if len(service_data) >= 14:
                            alt_raw = int.from_bytes(service_data[12:14], byteorder='little', signed=False)
                            # Altitude encoding: raw value represents meters without offset in this format
                            altitude = alt_raw
                            print(f"  Altitude: {altitude:.1f} m")
                            
                        if len(service_data) >= 20:
                            speed = service_data[19] * 0.25
                            print(f"  Speed: {speed:.2f} m/s")
                            
                # Subtype 0x12 = Another manufacturer location format with coordinates at bytes 7-14
                elif subtype == 0x12:
                    # This is location data with coordinates at bytes 7-14
                    if len(service_data) >= 15:
                        lat_raw = int.from_bytes(service_data[7:11], byteorder='little', signed=True)
                        lon_raw = int.from_bytes(service_data[11:15], byteorder='little', signed=True)
                        latitude = lat_raw * 1e-7
                        longitude = lon_raw * 1e-7
                        
                        print(f"  Type: Location (Manufacturer, 0x12)")
                        print(f"  Latitude: {latitude:.7f}°")
                        print(f"  Longitude: {longitude:.7f}°")
                        
                        # Parse additional telemetry if available
                        if len(service_data) >= 17:
                            alt_raw = int.from_bytes(service_data[15:17], byteorder='little', signed=False)
                            altitude = alt_raw
                            print(f"  Altitude: {altitude:.1f} m")
                            
                        if len(service_data) >= 22:
                            speed = service_data[21] * 0.25
                            print(f"  Speed: {speed:.2f} m/s")
                else:
                    print(f"  Type: {message_type}, Subtype: 0x{subtype:02x}")
            
                # Initialize device entry if not seen before
                if address not in detected_devices:
                    detected_devices[address] = {
                        "address": address,
                        "name": device.name or "Unknown",
                        "first_seen": datetime.now().isoformat(),
                        "service_uuids": service_uuids,
                        "basic_id": "",
                        "operator_id": "",
                        "messages": [],
                        "message_counts": {
                            "basic_id": 0,
                            "operator_id": 0,
                            "location": 0,
                            "total": 0
                        }
                    }
                    print(f"✓ Found Remote ID Device: {address} ({device.name or 'Unknown'})")
                
                # Update message counts
                detected_devices[address]["message_counts"]["total"] += 1
                
                # Build message data structure
                message_data = {
                    "timestamp": datetime.now().isoformat(),
                    "rssi": advertisement_data.rssi,
                    "raw_hex": service_data.hex(),
                    "message_type": message_type,
                    "subtype": subtype
                }
                
                # Parse based on message type and subtype
                if message_type == 0x02 and subtype == 0x02:
                    # Basic ID / Serial Number - store at device level
                    id_bytes = service_data[4:23] if len(service_data) >= 23 else service_data[4:]
                    basic_id = id_bytes.decode('ascii', errors='ignore').rstrip('\x00').strip()
                    if basic_id:
                        if detected_devices[address]["basic_id"] == "":
                            detected_devices[address]["basic_id"] = basic_id
                            print(f"  ✓ Captured Basic ID: {basic_id}")
                            
                            # Update database in real-time
                            if SAVE_TO_DB_REALTIME:
                                try:
                                    # Get or create drone RID record
                                    if address not in drone_rid_cache:
                                        drone_rid_cache[address] = getOrCreateDroneRid(
                                            address,
                                            detected_devices[address]["name"],
                                            detected_devices[address]["basic_id"],
                                            detected_devices[address]["operator_id"],
                                            list(service_uuids)
                                        )
                                    drone_rid_id = drone_rid_cache[address]
                                    
                                    # Update identity fields in database
                                    updateDroneRidIdentity(drone_rid_id, _basic_id=basic_id)
                                except Exception as e:
                                    print(f"  ⚠ DB update error: {e}")
                        detected_devices[address]["message_counts"]["basic_id"] += 1
                    
                elif message_type == 0x02 and subtype == 0x52:
                    # Operator ID - store at device level
                    # Byte 3 is often 0x00 (null), actual ID starts at byte 4
                    id_bytes = service_data[4:24] if len(service_data) >= 24 else service_data[4:]
                    operator_id = id_bytes.decode('ascii', errors='ignore').rstrip('\x00').strip()
                    if operator_id:
                        if detected_devices[address]["operator_id"] == "":
                            detected_devices[address]["operator_id"] = operator_id
                            print(f"  ✓ Captured Operator ID: {operator_id}")
                            
                            # Update database in real-time
                            if SAVE_TO_DB_REALTIME:
                                try:
                                    # Get or create drone RID record
                                    if address not in drone_rid_cache:
                                        drone_rid_cache[address] = getOrCreateDroneRid(
                                            address,
                                            detected_devices[address]["name"],
                                            detected_devices[address]["basic_id"],
                                            detected_devices[address]["operator_id"],
                                            list(service_uuids)
                                        )
                                    drone_rid_id = drone_rid_cache[address]
                                    
                                    # Update identity fields in database
                                    updateDroneRidIdentity(drone_rid_id, _operator_id=operator_id)
                                except Exception as e:
                                    print(f"  ⚠ DB update error: {e}")
                        detected_devices[address]["message_counts"]["operator_id"] += 1
                    
                elif subtype == 0x42:
                    # Location data (format 1: coordinates at bytes 4-11)
                    if len(service_data) >= 12:
                        lat_raw = int.from_bytes(service_data[4:8], byteorder='little', signed=True)
                        lon_raw = int.from_bytes(service_data[8:12], byteorder='little', signed=True)
                        message_data["latitude"] = lat_raw * 1e-7
                        message_data["longitude"] = lon_raw * 1e-7
                        
                        if len(service_data) >= 14:
                            alt_raw = int.from_bytes(service_data[12:14], byteorder='little', signed=False)
                            message_data["altitude_m"] = alt_raw
                            
                        if len(service_data) >= 20:
                            message_data["speed_m_s"] = service_data[19] * 0.25
                    
                    # Store location message
                    detected_devices[address]["messages"].append(message_data)
                    detected_devices[address]["message_counts"]["location"] += 1
                    
                    # Save to database in real-time
                    if SAVE_TO_DB_REALTIME and "latitude" in message_data:
                        try:
                            # Get or create drone RID record
                            if address not in drone_rid_cache:
                                drone_rid_cache[address] = getOrCreateDroneRid(
                                    address,
                                    detected_devices[address]["name"],
                                    detected_devices[address]["basic_id"],
                                    detected_devices[address]["operator_id"],
                                    list(service_uuids)
                                )
                            drone_rid_id = drone_rid_cache[address]
                            
                            # Save individual message
                            saveDroneRidMessage(drone_rid_id, message_data)
                            
                            # Update latest data on drone RID record
                            updateDroneRidLatestData(drone_rid_id, message_data)
                        except Exception as e:
                            print(f"  ⚠ DB save error: {e}")
                            
                elif subtype == 0x12:
                    # Location data (format 2: coordinates at bytes 7-14)
                    if len(service_data) >= 15:
                        lat_raw = int.from_bytes(service_data[7:11], byteorder='little', signed=True)
                        lon_raw = int.from_bytes(service_data[11:15], byteorder='little', signed=True)
                        message_data["latitude"] = lat_raw * 1e-7
                        message_data["longitude"] = lon_raw * 1e-7
                        
                        if len(service_data) >= 17:
                            alt_raw = int.from_bytes(service_data[15:17], byteorder='little', signed=False)
                            message_data["altitude_m"] = alt_raw
                            
                        if len(service_data) >= 22:
                            message_data["speed_m_s"] = service_data[21] * 0.25
                    
                    # Store location message
                    detected_devices[address]["messages"].append(message_data)
                    detected_devices[address]["message_counts"]["location"] += 1
                    
                    # Save to database in real-time
                    if SAVE_TO_DB_REALTIME and "latitude" in message_data:
                        try:
                            # Get or create drone RID record
                            if address not in drone_rid_cache:
                                drone_rid_cache[address] = getOrCreateDroneRid(
                                    address,
                                    detected_devices[address]["name"],
                                    detected_devices[address]["basic_id"],
                                    detected_devices[address]["operator_id"],
                                    list(service_uuids)
                                )
                            drone_rid_id = drone_rid_cache[address]
                            
                            # Save individual message
                            saveDroneRidMessage(drone_rid_id, message_data)
                            
                            # Update latest data on drone RID record
                            updateDroneRidLatestData(drone_rid_id, message_data)
                        except Exception as e:
                            print(f"  ⚠ DB save error: {e}")
                
                else:
                    # Store other unknown message types
                    detected_devices[address]["messages"].append(message_data)
            else:
                # Known beacon but no service data in this advertisement
                print(f"{address} - Known beacon (no RID data in this packet)")
            # detected_devices[address]["raw_data"].append(advertisement_data)
            # print(advertisement_data)
        else:
            # print(f"{device.address} - Ignored (no Remote ID service UUID)")
            pass
    
    # Start scanning with callback and run indefinitely
    scanner = BleakScanner(detection_callback=detection_callback)
    await scanner.start()
    
    try:
        # Keep running until interrupted
        while True:
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        print("\nStopping scanner...")
    finally:
        await scanner.stop()
        
        # Print final statistics
        if detected_devices:
            print("\n=== Final Statistics ===")
            for address, device_data in detected_devices.items():
                print(f"\nDevice: {address}")
                print(f"  Basic ID Messages: {device_data['message_counts']['basic_id']}")
                print(f"  Operator ID Messages: {device_data['message_counts']['operator_id']}")
                print(f"  Location Messages: {device_data['message_counts']['location']}")
                print(f"  Total Messages: {device_data['message_counts']['total']}")


async def scan_for_drone_remote_id(scan_duration=10, collect_messages=True):
    """
    Scan for BLE devices broadcasting the Remote ID service.
    
    Args:
        scan_duration (int): Duration to scan in seconds (default: 10)
        collect_messages (bool): Whether to collect messages from devices (default: True)    
    Returns:
        list: List of dictionaries containing Remote ID device information
    """
    if collect_messages:
        # Use continuous scanning to capture advertisement data
        detected_devices = await scan_for_drone_remote_id_continuous(scan_duration)
        
        # Display message capture statistics
        if detected_devices:
            print("\n=== Message Capture Statistics ===")
            for address, device_data in detected_devices.items():
                print(f"\nDevice: {address}")
                print(f"  Basic ID Messages: {device_data['message_counts']['basic_id']}")
                print(f"  Operator ID Messages: {device_data['message_counts']['operator_id']}")
                print(f"  Location Messages: {device_data['message_counts']['location']}")
                print(f"  Total Messages: {device_data['message_counts']['total']}")
        
        return list(detected_devices.values())
    else:
        # Quick scan without message collection
        print(f"Scanning for Drone Remote ID devices for {scan_duration} seconds...")
        print(f"Looking for service UUID: {REMOTE_ID_SERVICE_UUID}\n")
        
        devices = await BleakScanner.discover(timeout=scan_duration, return_adv=True)
        remote_id_devices = []
        
        for address, (device, advertisement_data) in devices.items():
            service_uuids = advertisement_data.service_uuids
            
            if REMOTE_ID_SERVICE_UUID in service_uuids:
                device_info = {
                    "address": address,
                    "name": device.name or "Unknown",
                    "rssi": advertisement_data.rssi,
                    "service_uuids": service_uuids,
                    "messages": []
                }
                
                print(f"✓ Found Remote ID Device:")
                print(f"  Address: {device_info['address']}")
                print(f"  Name: {device_info['name']}")
                print(f"  RSSI: {device_info['rssi']} dBm")
                print()
                
                remote_id_devices.append(device_info)
        
        return remote_id_devices


async def main():
    """Main function to run the scanner indefinitely."""

    load_dotenv()

    # initialize database connection pool
    poolCount = int(os.getenv('DRID_DB_CONNECTION_POOLS', 5))
    database.connection.init(poolCount, "dridPool")

    print(f"\n{'='*60}")
    print(f"Drone Remote ID Scanner - Continuous Mode")
    print(f"{'='*60}\n")
    
    # Run indefinite scan
    await scan_for_drone_remote_id_continuous()


if __name__ == "__main__":
    try:
        devices = asyncio.run(main())
    except KeyboardInterrupt:
        print("\nScan interrupted by user.")
    except Exception as e:
        print(f"Error: {e}")
        print("\nMake sure you have the required permissions to scan BLE devices.")
        print("On Linux, you may need to run with sudo or set capabilities:")
        print("  sudo setcap cap_net_raw,cap_net_admin+eip $(which python3)")
