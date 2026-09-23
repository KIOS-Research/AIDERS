#!/usr/bin/env python3
"""
Drone RID Generator and Simulator
Generates several simulated DroneRid entries and continuously updates their
positions and telemetry (location, altitude, speed, RSSI) with semi-realistic
movements including climbs, descents, and turning.

This script is designed to run inside the Django Docker container.
Usage: docker exec -it web python /app/_scripts/generate_drid_drones.py
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

from django.contrib.gis.geos import Point
from aiders.models.droneRid import DroneRid

# Drone type configurations
DRONE_TYPES = [
    {"model": "Quadcopter", "max_speed_m_s": 20.0, "climb_rate_m_s": 3.0, "turn_rate": 60.0},
    {"model": "Hexacopter", "max_speed_m_s": 18.0, "climb_rate_m_s": 2.5, "turn_rate": 50.0},
    {"model": "Fixed-wing", "max_speed_m_s": 35.0, "climb_rate_m_s": 4.0, "turn_rate": 20.0},
    {"model": "VTOL", "max_speed_m_s": 25.0, "climb_rate_m_s": 3.5, "turn_rate": 45.0},
]

STATUS_OPTIONS = ["Flying", "Hovering", "Landed", "Returning", "Lost Signal"]

# Starting area (Cyprus region as example - adjust as needed)
BASE_LAT = 35.1264
BASE_LON = 33.4299
AREA_RADIUS = 0.3  # degrees (~6.7 km)


class DroneSimulator:
    """Simulates a single DroneRid's movement and telemetry"""

    def __init__(self, drone: DroneRid, config: dict):
        self.drone = drone
        self.config = config
        self.target_speed = random.uniform(1.0, config['max_speed_m_s'] * 0.8)
        self.target_altitude = random.uniform(20.0, 120.0)
        self.turn_direction = 0  # -1 left, 0 straight, 1 right
        self.time_until_turn = random.uniform(3, 10)
        self.time_until_speed_change = random.uniform(2, 8)
        self.time_until_alt_change = random.uniform(4, 12)

    def update(self, delta_time: float):
        """Update drone telemetry and save model"""

        # Decide turns
        self.time_until_turn -= delta_time
        if self.time_until_turn <= 0:
            self.turn_direction = random.choice([-1, 0, 0, 1])
            self.time_until_turn = random.uniform(3, 10)

        # Decide speed changes
        self.time_until_speed_change -= delta_time
        if self.time_until_speed_change <= 0:
            self.target_speed = random.uniform(0.5, self.config['max_speed_m_s'] * 0.9)
            self.time_until_speed_change = random.uniform(2, 8)

        # Decide altitude changes
        self.time_until_alt_change -= delta_time
        if self.time_until_alt_change <= 0:
            self.target_altitude = random.uniform(10.0, 150.0)
            self.time_until_alt_change = random.uniform(4, 12)

        # Smoothly move speed toward target
        speed_diff = self.target_speed - (self.drone.speed_m_s or 0.0)
        if abs(speed_diff) > 0.01:
            accel = (self.config['max_speed_m_s'] * 0.2) * delta_time
            if speed_diff > 0:
                new_speed = min(self.target_speed, (self.drone.speed_m_s or 0.0) + accel)
            else:
                new_speed = max(self.target_speed, (self.drone.speed_m_s or 0.0) - accel)
            self.drone.speed_m_s = new_speed

        # Smooth altitude change
        alt_diff = self.target_altitude - (self.drone.altitude_m or 0.0)
        if abs(alt_diff) > 0.1:
            climb = self.config['climb_rate_m_s'] * delta_time
            if alt_diff > 0:
                self.drone.altitude_m = min(self.target_altitude, (self.drone.altitude_m or 0.0) + climb)
            else:
                self.drone.altitude_m = max(self.target_altitude, (self.drone.altitude_m or 0.0) - climb)

        # Heading
        if not hasattr(self.drone, 'heading') or self.drone.__dict__.get('heading', None) is None:
            # store heading on instance for simulation only
            self.drone.heading = random.uniform(0, 360)

        if self.turn_direction != 0:
            heading_change = self.turn_direction * self.config['turn_rate'] * delta_time
            self.drone.heading = (self.drone.heading + heading_change) % 360

        # Convert speed (m/s) to degrees per second approx: deg/s = m/s / 111320
        speed_deg_per_sec = (self.drone.speed_m_s or 0.0) / 111320.0

        heading_rad = math.radians(self.drone.heading)
        lat_change = speed_deg_per_sec * math.cos(heading_rad) * delta_time
        lon_change = (speed_deg_per_sec * math.sin(heading_rad) * delta_time) / math.cos(math.radians(self.drone.location.y if self.drone.location else BASE_LAT))

        # Update location
        lat = (self.drone.location.y if self.drone.location else BASE_LAT) + lat_change
        lon = (self.drone.location.x if self.drone.location else BASE_LON) + lon_change

        # Keep within area bounds
        if abs(lat - BASE_LAT) > AREA_RADIUS:
            self.drone.heading = (self.drone.heading + 180) % 360
            lat = BASE_LAT + (AREA_RADIUS if lat > BASE_LAT else -AREA_RADIUS)

        if abs(lon - BASE_LON) > AREA_RADIUS:
            self.drone.heading = (self.drone.heading + 180) % 360
            lon = BASE_LON + (AREA_RADIUS if lon > BASE_LON else -AREA_RADIUS)

        self.drone.location = Point(lon, lat, srid=4326)

        # RSSI - rough model: stronger when lower altitude and nearer base center
        dist_deg = math.hypot(lat - BASE_LAT, lon - BASE_LON)
        dist_m = dist_deg * 111320.0
        alt_factor = max(1.0, (self.drone.altitude_m or 0.0) / 50.0)
        base_rssi = -30 - (dist_m / 200.0) - (alt_factor * 5.0)
        self.drone.rssi = int(base_rssi + random.uniform(-3, 3))

        # Update message counts
        counts = self.drone.message_counts or {}
        counts['total'] = counts.get('total', 0) + 1
        counts['location'] = counts.get('location', 0) + 1
        self.drone.message_counts = counts

        # Occasionally change name/basic/operator
        if random.random() < 0.005:
            self.drone.name = f"Sim-{random.randint(1000,9999)}"

        # Randomly change status
        if random.random() < 0.01:
            self.drone.status = random.choice(STATUS_OPTIONS)

        # Save
        # Django's auto_now/auto_now_add will update last_seen
        self.drone.save()


def random_mac_address():
    return ':'.join(f"{random.randint(0,255):02X}" for _ in range(6))


def create_drones(count=8):
    """Create several DroneRid entries with random initial telemetry"""
    print("Creating DRID drones...")
    drones = []

    for i in range(count):
        addr = random_mac_address()
        name = f"DRID-SIM-{i+1:03d}"

        existing = DroneRid.objects.filter(address=addr).first()
        if existing:
            print(f"  Drone {addr} already exists, using existing")
            drone = existing
        else:
            dtype = random.choice(DRONE_TYPES)
            lat = BASE_LAT + random.uniform(-AREA_RADIUS, AREA_RADIUS)
            lon = BASE_LON + random.uniform(-AREA_RADIUS, AREA_RADIUS)
            drone = DroneRid(
                address=addr,
                name=name,
                basic_id=f"SIM-BASIC-{random.randint(1000,9999)}",
                operator_id=f"OP-{random.randint(100,999)}",
                service_uuids=[],
                location=Point(lon, lat, srid=4326),
                altitude_m=random.uniform(10.0, 120.0),
                speed_m_s=random.uniform(0.5, 10.0),
                rssi=random.randint(-80, -30),
                message_counts={"total": 1, "location": 1}
            )
            drone.save()
            print(f"  Created {addr} ({dtype['model']})")

        # attach a runtime config tuple for simulation
        dtype = random.choice(DRONE_TYPES)
        drones.append((drone, dtype))

    print(f"\nTotal drones: {len(drones)}")
    return drones


def simulate_drones(drones, duration=None, update_interval=1.0):
    """Simulate drone movements

    Args:
        drones: List of (drone, config) tuples
        duration: Total duration in seconds (None for infinite)
        update_interval: Time between updates in seconds
    """
    print(f"\nStarting DRID simulation (update every {update_interval}s)...")
    print("Press Ctrl+C to stop\n")

    simulators = [DroneSimulator(drone, config) for drone, config in drones]

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
                        d = sim.drone
                        lat = d.location.y if d.location else 0.0
                        lon = d.location.x if d.location else 0.0
                        print(f"  {d.address}: Pos({lat:.5f}, {lon:.5f}) Alt={d.altitude_m:.1f}m Speed={d.speed_m_s:.1f}m/s RSSI={d.rssi} Status={getattr(d, 'status', 'Unknown')}")

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
    print("Drone RID Generator & Simulator")
    print("=" * 60)

    drones = create_drones(count=8)

    try:
        simulate_drones(drones, update_interval=0.5)
    except Exception as e:
        print(f"\nError during simulation: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
