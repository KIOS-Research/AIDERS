#!/usr/bin/env python3
"""
Ground Vehicle Generator and Simulator
Generates 10 random ground vehicles and continuously updates their positions
with semi-realistic movements including acceleration, deceleration, and turning.

This script is designed to run inside the Django Docker container.
Usage: docker exec -it web python /app/_scripts/generate_ground_vehicles.py
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

from aiders.models.groundVehicle import GroundVehicle

# Vehicle type configurations with realistic parameters
VEHICLE_TYPES = [
    {"model": "Police Cruiser", "service": "POLICE", "max_speed": 120, "accel_rate": 0.8, "turn_rate": 5.0},
    {"model": "Ambulance", "service": "FIRE", "max_speed": 100, "accel_rate": 0.7, "turn_rate": 4.0},
    {"model": "Fire Truck", "service": "AMBULANCE", "max_speed": 90, "accel_rate": 0.5, "turn_rate": 3.0},
    {"model": "Utility Van", "service": "AMBULANCE", "max_speed": 80, "accel_rate": 0.6, "turn_rate": 4.5},
    {"model": "Search & Rescue", "service": "FIRE", "max_speed": 85, "accel_rate": 0.65, "turn_rate": 4.0},
    {"model": "Mobile Command Unit", "service": "Command", "max_speed": 70, "accel_rate": 0.4, "turn_rate": 2.5},
    {"model": "Patrol SUV", "service": "Police", "max_speed": 110, "accel_rate": 0.75, "turn_rate": 4.5},
    {"model": "Emergency Response", "service": "Emergency", "max_speed": 95, "accel_rate": 0.7, "turn_rate": 4.5},
    {"model": "Hazmat Vehicle", "service": "Hazmat", "max_speed": 75, "accel_rate": 0.5, "turn_rate": 3.0},
    {"model": "Security Patrol", "service": "Security", "max_speed": 90, "accel_rate": 0.65, "turn_rate": 4.0},
]

STATUS_OPTIONS = ["Active", "Patrolling", "Responding", "On Scene", "En Route", "Available"]

# Starting area (Cyprus region as example - adjust as needed)
BASE_LAT = 35.1264
BASE_LON = 33.4299
AREA_RADIUS = 0.05  # degrees (~5.5 km)

class VehicleSimulator:
    """Manages vehicle state and movement simulation"""
    
    def __init__(self, vehicle, config):
        self.vehicle = vehicle
        self.config = config
        self.target_speed = random.uniform(30, config['max_speed'] * 0.7)
        self.turn_direction = 0  # -1 left, 0 straight, 1 right
        self.time_until_turn = random.uniform(5, 15)  # seconds until next turn decision
        self.time_until_speed_change = random.uniform(3, 10)
        
    def update(self, delta_time):
        """Update vehicle position and state based on elapsed time"""
        
        # Update turn timing
        self.time_until_turn -= delta_time
        if self.time_until_turn <= 0:
            self.turn_direction = random.choice([-1, 0, 0, 0, 1])  # Bias toward straight
            self.time_until_turn = random.uniform(5, 15)
        
        # Update speed timing
        self.time_until_speed_change -= delta_time
        if self.time_until_speed_change <= 0:
            self.target_speed = random.uniform(20, self.config['max_speed'] * 0.8)
            self.time_until_speed_change = random.uniform(3, 10)
        
        # Accelerate/decelerate toward target speed
        speed_diff = self.target_speed - self.vehicle.speed
        if abs(speed_diff) > 0.1:
            accel = self.config['accel_rate'] * delta_time
            if speed_diff > 0:
                self.vehicle.speed = min(self.target_speed, self.vehicle.speed + accel)
            else:
                self.vehicle.speed = max(self.target_speed, self.vehicle.speed - accel)
        
        # Update heading based on turn direction
        if self.turn_direction != 0:
            heading_change = self.turn_direction * self.config['turn_rate'] * delta_time
            self.vehicle.heading = (self.vehicle.heading + heading_change) % 360
        
        # Calculate position change
        # Convert speed from km/h to degrees per second (approximate)
        speed_deg_per_sec = (self.vehicle.speed / 3600) / 111.32  # 1 degree ~= 111.32 km
        
        # Calculate new position
        heading_rad = math.radians(self.vehicle.heading)
        lat_change = speed_deg_per_sec * math.cos(heading_rad) * delta_time
        lon_change = speed_deg_per_sec * math.sin(heading_rad) * delta_time / math.cos(math.radians(self.vehicle.lat))
        
        self.vehicle.lat += lat_change
        self.vehicle.lon += lon_change
        
        # Keep vehicles within bounds (bounce back)
        if abs(self.vehicle.lat - BASE_LAT) > AREA_RADIUS:
            self.vehicle.heading = (self.vehicle.heading + 180) % 360
            self.vehicle.lat = BASE_LAT + (AREA_RADIUS if self.vehicle.lat > BASE_LAT else -AREA_RADIUS)
        
        if abs(self.vehicle.lon - BASE_LON) > AREA_RADIUS:
            self.vehicle.heading = (self.vehicle.heading + 180) % 360
            self.vehicle.lon = BASE_LON + (AREA_RADIUS if self.vehicle.lon > BASE_LON else -AREA_RADIUS)
        
        # Occasionally update status
        if random.random() < 0.01:  # 1% chance per update
            self.vehicle.status = random.choice(STATUS_OPTIONS)
        
        # Save to database
        self.vehicle.save()


def create_vehicles():
    """Create 10 ground vehicles with random initial positions"""
    print("Creating ground vehicles...")
    vehicles = []
    
    for i, vtype in enumerate(VEHICLE_TYPES):
        name = f"GVS-{i+1:03d}"
        
        # Check if vehicle already exists
        existing = GroundVehicle.objects.filter(name=name).first()
        if existing:
            print(f"  Vehicle {name} already exists, using existing")
            vehicle = existing
        else:
            # Create new vehicle
            vehicle = GroundVehicle(
                name=name,
                model=vtype['model'],
                service=vtype['service'],
                lat=BASE_LAT + random.uniform(-AREA_RADIUS, AREA_RADIUS),
                lon=BASE_LON + random.uniform(-AREA_RADIUS, AREA_RADIUS),
                speed=random.uniform(20, 60),
                heading=random.uniform(0, 360),
                status=random.choice(STATUS_OPTIONS)
            )
            vehicle.save()
            print(f"  Created {name} - {vtype['model']}")
        
        vehicles.append((vehicle, vtype))
    
    print(f"\nTotal vehicles: {len(vehicles)}")
    return vehicles


def simulate_vehicles(vehicles, duration=None, update_interval=1.0):
    """
    Simulate vehicle movements
    
    Args:
        vehicles: List of (vehicle, config) tuples
        duration: Total duration in seconds (None for infinite)
        update_interval: Time between updates in seconds
    """
    print(f"\nStarting simulation (update every {update_interval}s)...")
    print("Press Ctrl+C to stop\n")
    
    # Create simulators
    simulators = [VehicleSimulator(vehicle, config) for vehicle, config in vehicles]
    
    start_time = time.time()
    last_update = start_time
    update_count = 0
    
    try:
        while True:
            current_time = time.time()
            delta_time = current_time - last_update
            
            if delta_time >= update_interval:
                # Update all vehicles
                for sim in simulators:
                    sim.update(delta_time)
                
                update_count += 1
                elapsed = current_time - start_time
                
                # Print status
                if update_count % 10 == 0:
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] "
                          f"Update #{update_count} - Elapsed: {elapsed:.1f}s")
                    for sim in simulators[:3]:  # Show first 3 vehicles
                        v = sim.vehicle
                        print(f"  {v.name}: Pos({v.lat:.5f}, {v.lon:.5f}) "
                              f"Speed={v.speed:.1f}km/h Heading={v.heading:.1f}° Status={v.status}")
                
                last_update = current_time
                
                # Check duration
                if duration and elapsed >= duration:
                    print(f"\nSimulation completed after {elapsed:.1f}s")
                    break
            
            time.sleep(0.1)  # Small sleep to prevent CPU spinning
            
    except KeyboardInterrupt:
        print("\n\nSimulation stopped by user")
        elapsed = time.time() - start_time
        print(f"Total updates: {update_count}")
        print(f"Total time: {elapsed:.1f}s")


def main():
    """Main entry point"""
    print("=" * 60)
    print("Ground Vehicle Generator & Simulator")
    print("=" * 60)
    
    # Create vehicles
    vehicles = create_vehicles()
    
    # Run simulation
    try:
        simulate_vehicles(vehicles, update_interval=0.5)
    except Exception as e:
        print(f"\nError during simulation: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
