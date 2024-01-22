import asyncio
from mavsdk import System, telemetry, offboard
# from utm import from_latlon
# from mavsdk import (OffboardError, OffboardMode)

async def run():
    # Connect to the drone
    drone = System()
    await drone.connect(system_address="udp://:14445")


    try:
        # Arm the drone
        print("Arming the drone...")
        await drone.action.arm()

        # Wait for arming confirmation
        async for is_armed in drone.telemetry.armed():
            if is_armed:
                print("Drone is armed.")
                break

        # Define the take-off parameters
        latitude = 35.02782821 # Replace with the desired latitude
        longitude = 33.35619363 # Replace with the desired longitude
        altitude = 9.0  # Replace with the desired altitude above ground

        # Send the take-off command
        print("Sending take-off command...")
        await drone.action.takeoff()
        await asyncio.sleep(2)  # Allow time for the take-off command to be executed

        # await drone.offboard.set_position_ned(offboard.PositionNedYaw(5.0, 10.0, 15.0, -20.0))
        # await drone.offboard.start()

        # Check the current position
        async for position in drone.telemetry.position():
            if position.relative_altitude_m > 0.9 * altitude:
                print("Drone has taken off.")
                break

        # Perform other operations or mission tasks here
        print(f"Sending the drone to coordinates: Lat={latitude} Lon={longitude} Alt={altitude}")
        # TODO: move the drone to a specific location and altitude
        print("!! TODO TODO !!")
        target_position = [10.0, 10.0, -10.0]  # [North, East, Down] coordinates
        # await drone.offboard.set_position_ned(target_position[0], target_position[1], target_position[2])
        await drone.offboard.set_position_ned(offboard.PositionNedYaw(5.0, 10.0, 32.0, 270.0))
        await drone.offboard.start()


        # Land the drone
        print("Landing the drone...")
        await drone.action.land()

        # Wait for the drone to land
        async for is_in_air in drone.telemetry.in_air():
            if not is_in_air:
                print("Drone has landed.")
                break

    finally:
        pass

async def main():
    await run()

if __name__ == "__main__":
    asyncio.run(main())
