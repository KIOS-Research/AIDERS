#!/usr/bin/env python3
"""
ADS-B Aircraft Generator and Simulator
Generates several simulated AdsbAircraft entries and continuously updates their
positions and telemetry (location, altitude, ground speed, track, vertical rate)
with semi-realistic movements including climbs, descents, and turning. Also
writes periodic AdsbAircraftHistory points so the trail endpoint has data.

This script is designed to run inside the Django Docker container.
Usage: docker exec -it web python /app/_scripts/generate_adsb_aircraft.py
"""

import os
import sys
import django
import random
import math
import time
from datetime import datetime

# Setup Django environment
sys.path.insert(0, '/app')
os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'django_api.settings')
django.setup()

from django.utils import timezone
from django.contrib.gis.geos import Point
from aiders.models.adsbAircraft import AdsbAircraft, AdsbAircraftHistory

# Aircraft type configurations (speeds in knots, climb rates in ft/min, turn rate in deg/s)
AIRCRAFT_TYPES = [
    {"model": "A320", "cruise_speed_kts": 450.0, "climb_rate_fpm": 2000.0, "turn_rate": 3.0, "cruise_alt_ft": 35000.0},
    {"model": "B738", "cruise_speed_kts": 460.0, "climb_rate_fpm": 2200.0, "turn_rate": 3.0, "cruise_alt_ft": 37000.0},
    {"model": "E190", "cruise_speed_kts": 420.0, "climb_rate_fpm": 1800.0, "turn_rate": 3.5, "cruise_alt_ft": 33000.0},
    {"model": "C172", "cruise_speed_kts": 110.0, "climb_rate_fpm": 700.0, "turn_rate": 6.0, "cruise_alt_ft": 6500.0},
    {"model": "PC12", "cruise_speed_kts": 270.0, "climb_rate_fpm": 1500.0, "turn_rate": 4.0, "cruise_alt_ft": 28000.0},
]

SQUAWK_OPTIONS = ["1000", "2000", "7000", "0421", "6512", "3401"]

# Starting area (Cyprus FIR bounds, matching views_adsb.py)
CYPRUS_MIN_LAT = 34.30
CYPRUS_MAX_LAT = 35.10
CYPRUS_MIN_LON = 32.10
CYPRUS_MAX_LON = 34.60
BASE_LAT = (CYPRUS_MIN_LAT + CYPRUS_MAX_LAT) / 2
BASE_LON = (CYPRUS_MIN_LON + CYPRUS_MAX_LON) / 2

KTS_TO_MPS = 0.514444
FT_TO_M = 0.3048

HISTORY_INTERVAL = 30.0  # seconds between AdsbAircraftHistory snapshots


class AircraftSimulator:
    """Simulates a single AdsbAircraft's movement and telemetry"""

    def __init__(self, aircraft: AdsbAircraft, config: dict):
        self.aircraft = aircraft
        self.config = config
        self.target_speed = random.uniform(config['cruise_speed_kts'] * 0.7, config['cruise_speed_kts'])
        self.target_altitude = random.uniform(config['cruise_alt_ft'] * 0.6, config['cruise_alt_ft'])
        self.heading = random.uniform(0, 360)
        self.turn_direction = 0  # -1 left, 0 straight, 1 right
        self.time_until_turn = random.uniform(10, 30)
        self.time_until_speed_change = random.uniform(15, 40)
        self.time_until_alt_change = random.uniform(20, 60)
        self.time_since_history = 0.0

    def update(self, delta_time: float):
        """Update aircraft telemetry and save model"""

        # Decide turns
        self.time_until_turn -= delta_time
        if self.time_until_turn <= 0:
            self.turn_direction = random.choice([-1, 0, 0, 1])
            self.time_until_turn = random.uniform(10, 30)

        # Decide speed changes
        self.time_until_speed_change -= delta_time
        if self.time_until_speed_change <= 0:
            self.target_speed = random.uniform(self.config['cruise_speed_kts'] * 0.6, self.config['cruise_speed_kts'])
            self.time_until_speed_change = random.uniform(15, 40)

        # Decide altitude changes
        self.time_until_alt_change -= delta_time
        if self.time_until_alt_change <= 0:
            self.target_altitude = random.uniform(self.config['cruise_alt_ft'] * 0.4, self.config['cruise_alt_ft'])
            self.time_until_alt_change = random.uniform(20, 60)

        # Smoothly move speed toward target
        current_speed = self.aircraft.ground_speed_kts or 0.0
        speed_diff = self.target_speed - current_speed
        if abs(speed_diff) > 0.5:
            accel = (self.config['cruise_speed_kts'] * 0.05) * delta_time
            if speed_diff > 0:
                new_speed = min(self.target_speed, current_speed + accel)
            else:
                new_speed = max(self.target_speed, current_speed - accel)
            self.aircraft.ground_speed_kts = int(new_speed)

        # Smooth altitude change, tracking vertical rate
        current_altitude = self.aircraft.altitude_ft or 0.0
        alt_diff = self.target_altitude - current_altitude
        climb = (self.config['climb_rate_fpm'] / 60.0) * delta_time
        if abs(alt_diff) > 10.0:
            if alt_diff > 0:
                new_altitude = min(self.target_altitude, current_altitude + climb)
                self.aircraft.vertical_rate_fpm = int(self.config['climb_rate_fpm'])
            else:
                new_altitude = max(self.target_altitude, current_altitude - climb)
                self.aircraft.vertical_rate_fpm = -int(self.config['climb_rate_fpm'])
            self.aircraft.altitude_ft = int(new_altitude)
        else:
            self.aircraft.vertical_rate_fpm = 0

        # Heading
        if self.turn_direction != 0:
            heading_change = self.turn_direction * self.config['turn_rate'] * delta_time
            self.heading = (self.heading + heading_change) % 360
        self.aircraft.track_deg = self.heading

        # Convert speed (knots) to degrees per second approx
        speed_mps = (self.aircraft.ground_speed_kts or 0.0) * KTS_TO_MPS
        speed_deg_per_sec = speed_mps / 111320.0

        heading_rad = math.radians(self.heading)
        lat = self.aircraft.latitude if self.aircraft.latitude is not None else BASE_LAT
        lon = self.aircraft.longitude if self.aircraft.longitude is not None else BASE_LON

        lat_change = speed_deg_per_sec * math.cos(heading_rad) * delta_time
        lon_change = (speed_deg_per_sec * math.sin(heading_rad) * delta_time) / math.cos(math.radians(lat))

        lat += lat_change
        lon += lon_change

        # Keep within Cyprus FIR bounds, bouncing back in when hitting an edge
        if lat < CYPRUS_MIN_LAT or lat > CYPRUS_MAX_LAT:
            self.heading = (self.heading + 180) % 360
            lat = min(max(lat, CYPRUS_MIN_LAT), CYPRUS_MAX_LAT)

        if lon < CYPRUS_MIN_LON or lon > CYPRUS_MAX_LON:
            self.heading = (self.heading + 180) % 360
            lon = min(max(lon, CYPRUS_MIN_LON), CYPRUS_MAX_LON)

        self.aircraft.latitude = lat
        self.aircraft.longitude = lon
        self.aircraft.location = Point(lon, lat, srid=4326)

        # Occasionally change squawk
        if random.random() < 0.002:
            self.aircraft.squawk = random.choice(SQUAWK_OPTIONS)

        # Rarely toggle emergency (kept low probability)
        if random.random() < 0.0005:
            self.aircraft.emergency = not self.aircraft.emergency

        self.aircraft.on_ground = False

        # Save
        # Django's auto_now will update last_seen
        self.aircraft.save()

        # Periodically record a history point
        self.time_since_history += delta_time
        if self.time_since_history >= HISTORY_INTERVAL:
            self.time_since_history = 0.0
            AdsbAircraftHistory.objects.create(
                aircraft=self.aircraft,
                timestamp=timezone.now(),
                location=self.aircraft.location,
                latitude=self.aircraft.latitude,
                longitude=self.aircraft.longitude,
                altitude_ft=self.aircraft.altitude_ft,
                ground_speed_kts=self.aircraft.ground_speed_kts,
                track_deg=self.aircraft.track_deg,
                vertical_rate_fpm=self.aircraft.vertical_rate_fpm,
            )


def random_icao24():
    return ''.join(random.choice("0123456789ABCDEF") for _ in range(6))


def random_callsign():
    prefixes = ["CYP", "AEE", "THY", "UAE", "BAW", "RJA", "MSR"]
    return f"{random.choice(prefixes)}{random.randint(100, 9999)}"


def create_aircraft(count=8):
    """Create several AdsbAircraft entries with random initial telemetry"""
    print("Creating ADS-B aircraft...")
    aircraft_list = []

    for i in range(count):
        icao = random_icao24()

        existing = AdsbAircraft.objects.filter(icao24=icao).first()
        if existing:
            print(f"  Aircraft {icao} already exists, using existing")
            aircraft = existing
        else:
            atype = random.choice(AIRCRAFT_TYPES)
            lat = CYPRUS_MIN_LAT + random.uniform(0, CYPRUS_MAX_LAT - CYPRUS_MIN_LAT)
            lon = CYPRUS_MIN_LON + random.uniform(0, CYPRUS_MAX_LON - CYPRUS_MIN_LON)
            aircraft = AdsbAircraft(
                icao24=icao,
                callsign=random_callsign(),
                location=Point(lon, lat, srid=4326),
                latitude=lat,
                longitude=lon,
                altitude_ft=int(random.uniform(atype['cruise_alt_ft'] * 0.5, atype['cruise_alt_ft'])),
                ground_speed_kts=int(random.uniform(atype['cruise_speed_kts'] * 0.6, atype['cruise_speed_kts'])),
                track_deg=random.uniform(0, 360),
                vertical_rate_fpm=0,
                squawk=random.choice(SQUAWK_OPTIONS),
                emergency=False,
                on_ground=False,
            )
            aircraft.save()
            print(f"  Created {icao} ({aircraft.callsign}, {atype['model']})")

        atype = random.choice(AIRCRAFT_TYPES)
        aircraft_list.append((aircraft, atype))

    print(f"\nTotal aircraft: {len(aircraft_list)}")
    return aircraft_list


def simulate_aircraft(aircraft_list, duration=None, update_interval=1.0):
    """Simulate aircraft movements

    Args:
        aircraft_list: List of (aircraft, config) tuples
        duration: Total duration in seconds (None for infinite)
        update_interval: Time between updates in seconds
    """
    print(f"\nStarting ADS-B simulation (update every {update_interval}s)...")
    print("Press Ctrl+C to stop\n")

    simulators = [AircraftSimulator(aircraft, config) for aircraft, config in aircraft_list]

    start_time = time.time()
    last_update = start_time
    update_count = 0

    try:
        while True:
            current_time = time.time()
            delta_time = current_time - last_update

            if delta_time >= update_interval:
                for sim in simulators:
                    sim.update(delta_time)

                update_count += 1
                elapsed = current_time - start_time

                if update_count % 10 == 0:
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] Update #{update_count} - Elapsed: {elapsed:.1f}s")
                    for sim in simulators[:4]:
                        a = sim.aircraft
                        print(f"  {a.icao24} ({a.callsign}): Pos({a.latitude:.5f}, {a.longitude:.5f}) "
                              f"Alt={a.altitude_ft}ft Speed={a.ground_speed_kts}kts Track={a.track_deg:.0f} "
                              f"VS={a.vertical_rate_fpm}fpm Squawk={a.squawk}")

                last_update = current_time

                if duration and elapsed >= duration:
                    print(f"\nSimulation completed after {elapsed:.1f}s")
                    break

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\nSimulation stopped by user")
        elapsed = time.time() - start_time
        print(f"Total updates: {update_count}")
        print(f"Total time: {elapsed:.1f}s")


def main():
    print("=" * 60)
    print("ADS-B Aircraft Generator & Simulator")
    print("=" * 60)

    aircraft_list = create_aircraft(count=8)

    try:
        simulate_aircraft(aircraft_list, update_interval=0.5)
    except Exception as e:
        print(f"\nError during simulation: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
