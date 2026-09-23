#!/usr/bin/env python3
"""
ADS-B Scanner using dump1090
Connects to dump1090 via network socket to receive aircraft data in JSON format
and saves it to the database.
"""

import os
import socket
import json
import time
from datetime import datetime
from dotenv import load_dotenv
import pytz

# Custom database imports
import database.connection
from database.queries import saveOrUpdateAircraft, getAircraftByIcao

# Load environment variables
load_dotenv()

# Configuration
DUMP1090_HOST = os.environ.get('DUMP1090_HOST', 'localhost')
DUMP1090_PORT = int(os.environ.get('DUMP1090_PORT', 30003))  # SBS BaseStation format
RECONNECT_DELAY = 5  # seconds
SAVE_TO_DB_REALTIME = True

# Timezone
timezone = pytz.utc


def parse_sbs_message(line):
    """
    Parse SBS BaseStation format message from dump1090.
    Format: MSG,transmission_type,session_id,aircraft_id,icao24,flight_id,
            date_generated,time_generated,date_logged,time_logged,
            callsign,altitude,groundspeed,track,lat,lon,vertical_rate,
            squawk,alert,emergency,spi,is_on_ground
    
    Returns dict with parsed data or None if invalid
    """
    try:
        parts = line.strip().split(',')
        
        if len(parts) < 22 or parts[0] != 'MSG':
            return None
        
        msg_type = parts[1]
        icao24 = parts[4].strip()
        
        if not icao24:
            return None
        
        result = {
            'icao24': icao24,
            'msg_type': msg_type,
            'timestamp': datetime.now(timezone)
        }
        
        # Callsign (field 10)
        if parts[10].strip():
            result['callsign'] = parts[10].strip()
        
        # Altitude in feet (field 11)
        if parts[11].strip():
            try:
                result['altitude_ft'] = int(parts[11].strip())
            except ValueError:
                pass
        
        # Ground speed in knots (field 12)
        if parts[12].strip():
            try:
                result['ground_speed_kts'] = int(parts[12].strip())
            except ValueError:
                pass
        
        # Track/heading in degrees (field 13)
        if parts[13].strip():
            try:
                result['track_deg'] = float(parts[13].strip())
            except ValueError:
                pass
        
        # Latitude (field 14)
        if parts[14].strip():
            try:
                result['latitude'] = float(parts[14].strip())
            except ValueError:
                pass
        
        # Longitude (field 15)
        if parts[15].strip():
            try:
                result['longitude'] = float(parts[15].strip())
            except ValueError:
                pass
        
        # Vertical rate in feet/min (field 16)
        if parts[16].strip():
            try:
                result['vertical_rate_fpm'] = int(parts[16].strip())
            except ValueError:
                pass
        
        # Squawk code (field 17)
        if parts[17].strip():
            result['squawk'] = parts[17].strip()
        
        # Emergency flag (field 19)
        if parts[19].strip() and parts[19].strip() != '0':
            result['emergency'] = True
        
        # On ground flag (field 21)
        if parts[21].strip() and parts[21].strip() != '0':
            result['on_ground'] = True
        
        return result
        
    except Exception as e:
        print(f"Error parsing SBS message: {e}")
        return None


def connect_to_dump1090():
    """Establish connection to dump1090 network socket"""
    while True:
        try:
            print(f"Connecting to dump1090 at {DUMP1090_HOST}:{DUMP1090_PORT}...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)  # 5 second timeout for connection attempt
            sock.connect((DUMP1090_HOST, DUMP1090_PORT))
            sock.settimeout(None)  # Remove timeout after successful connection
            print("✓ Connected to dump1090")
            return sock
        except socket.timeout:
            print(f"⚠ Connection timeout - dump1090 may not be running")
            print(f"  This usually means no RTL-SDR antenna is connected")
            print(f"  Retrying in {RECONNECT_DELAY} seconds...")
            time.sleep(RECONNECT_DELAY)
        except ConnectionRefusedError:
            print(f"⚠ Connection refused - dump1090 is not running or not listening on port {DUMP1090_PORT}")
            print(f"  Check if RTL-SDR antenna is connected")
            print(f"  Retrying in {RECONNECT_DELAY} seconds...")
            time.sleep(RECONNECT_DELAY)
        except Exception as e:
            print(f"⚠ Connection failed: {e}")
            print(f"  Check antenna connection and dump1090 status")
            print(f"  Retrying in {RECONNECT_DELAY} seconds...")
            time.sleep(RECONNECT_DELAY)


def process_aircraft_data():
    """
    Main loop to receive and process aircraft data from dump1090.
    Handles antenna connection/disconnection gracefully.
    """
    print("Starting ADS-B Scanner...")
    print(f"Target: dump1090 at {DUMP1090_HOST}:{DUMP1090_PORT}")
    print(f"Database saving: {'ENABLED' if SAVE_TO_DB_REALTIME else 'DISABLED'}")
    print(f"\n⚠ Note: If no RTL-SDR antenna is connected, the scanner will keep retrying.")
    print(f"   You can plug in the antenna at any time.\n")
    
    # Initialize database connection
    database.connection.init(1, "adsb_pool")
    
    sock = None
    buffer = ""
    aircraft_cache = {}  # Cache to track aircraft updates
    connection_attempts = 0
    
    try:
        while True:
            try:
                # Establish connection if not connected
                if sock is None:
                    sock = connect_to_dump1090()
                    buffer = ""  # Reset buffer on new connection
                    connection_attempts = 0
                    print("✓ Ready to receive aircraft data")
                
                # Receive data from socket
                data = sock.recv(4096).decode('utf-8', errors='ignore')
                
                if not data:
                    print("⚠ Connection closed by dump1090")
                    print("  This may happen if the antenna was disconnected")
                    print("  Attempting to reconnect...")
                    if sock:
                        sock.close()
                    sock = None
                    continue
                
                buffer += data
                
                # Process complete lines
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    
                    if not line.strip():
                        continue
                    
                    # Parse the message
                    aircraft_data = parse_sbs_message(line)
                    
                    if aircraft_data:
                        icao24 = aircraft_data['icao24']
                        
                        # Update cache
                        if icao24 not in aircraft_cache:
                            aircraft_cache[icao24] = {}
                        
                        aircraft_cache[icao24].update(aircraft_data)
                        
                        # Print aircraft info
                        print(f"\n[{aircraft_data['timestamp'].strftime('%H:%M:%S')}] Aircraft: {icao24}")
                        if 'callsign' in aircraft_data:
                            print(f"  Callsign: {aircraft_data['callsign']}")
                        if 'latitude' in aircraft_data and 'longitude' in aircraft_data:
                            print(f"  Position: {aircraft_data['latitude']:.6f}, {aircraft_data['longitude']:.6f}")
                        if 'altitude_ft' in aircraft_data:
                            print(f"  Altitude: {aircraft_data['altitude_ft']} ft")
                        if 'ground_speed_kts' in aircraft_data:
                            print(f"  Speed: {aircraft_data['ground_speed_kts']} kts")
                        if 'track_deg' in aircraft_data:
                            print(f"  Track: {aircraft_data['track_deg']}°")
                        
                        # Save to database
                        if SAVE_TO_DB_REALTIME:
                            try:
                                saveOrUpdateAircraft(aircraft_cache[icao24])
                                print(f"  ✓ Saved to database")
                            except Exception as e:
                                print(f"  ✗ Database error: {e}")
                
            except socket.timeout:
                # Timeout on receive - connection may be dead
                print("⚠ Socket timeout - connection may be lost")
                if sock:
                    sock.close()
                sock = None
                
            except socket.error as e:
                print(f"⚠ Socket error: {e}")
                print("  Reconnecting...")
                if sock:
                    sock.close()
                sock = None
                buffer = ""
            
            except KeyboardInterrupt:
                raise
            
            except Exception as e:
                print(f"⚠ Error processing data: {e}")
                time.sleep(1)
    
    except KeyboardInterrupt:
        print("\n\nShutting down ADS-B Scanner...")
        if sock:
            sock.close()
    
    except Exception as e:
        print(f"⚠ Fatal error: {e}")
        if sock:
            sock.close()


if __name__ == "__main__":
    try:
        process_aircraft_data()
    except KeyboardInterrupt:
        print("\nExiting...")
