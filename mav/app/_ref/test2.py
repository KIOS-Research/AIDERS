#!/usr/bin/env python3


import asyncio
from mavsdk import System


async def run():

    drone = System()
    # await drone.connect(system_address="udp://:14550")
    await drone.connect(system_address="udp://:14445")

    status_text_task = asyncio.ensure_future(print_status_text(drone))

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("Fetching amsl altitude at home location....")
    async for terrain_info in drone.telemetry.home():
        absolute_altitude = terrain_info.absolute_altitude_m
        break


    print("-- Arming")
    await drone.action.arm()

    await drone.action.set_takeoff_altitude(11.0)
    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(20)

    await drone.action.transition_to_fixedwing()

    # To fly drone 20m above the ground plane
    flying_alt = absolute_altitude + 20.0
    # goto_location() takes Absolute MSL altitude
    await drone.action.goto_location(35.039041, 33.345370, flying_alt, 120)

    # print("-- Landing")
    # await drone.action.land()

    # await asyncio.sleep(10)

    status_text_task.cancel()


async def print_status_text(drone):
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())