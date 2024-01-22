import asyncio
from mavsdk import System
import time
import mavsdk.action

readingSecondaryInput = False

async def arm_drone(drone):
    print("Arming the drone...")
    try:
        await drone.action.arm()

        # Wait for arming confirmation
        async for is_armed in drone.telemetry.armed():
            if is_armed:
                print("Drone is armed.")
                break
    except:
        print("ERROR: Arming failed!")

async def disarm_drone(drone):
    print("Disarming the drone...")
    try:
        await drone.action.disarm()

        # Wait for arming confirmation
        async for is_armed in drone.telemetry.armed():
            if not is_armed:
                print("Drone is disarmed.")
                break
    except:
        print("ERROR: Disarming failed!")

async def takeoff_drone(drone):
    # if drone.telemetry.in_air():
    #     print("Drone is already flying...")
    #     print(drone.telemetry.in_air())
    #     return
    # user_input = input("AAAAAAAAAAAAAAAAAAA\n")
    readingSecondaryInput = True
    altitude = await asyncio.get_event_loop().run_in_executor(None, input, 'Takeoff Altitude: ')
    await arm_drone(drone)
    print("Taking off...")
    try:
        print("AAAAAAAAAAAAAAAAAAAAAAAAAAAALT: " + altitude)
        await drone.action.set_takeoff_altitude(int(altitude))
        # time.sleep(2)
        await drone.action.takeoff()

        # Wait for the drone to be in the air
        async for is_in_air in drone.telemetry.in_air():
            if is_in_air:
                print("Drone has taken off.")
                readingSecondaryInput = False
                # await asyncio.sleep(20)
                # await drone.action.transition_to_fixedwing()
                break
    except:
        print("ERROR: Takeoff failed!")

async def transition_fw(drone):
    print("Transitioning to Fixed Wing...")
    try:
        await drone.action.transition_to_fixedwing()
    except:
        print("ERROR: Transitioning to Fixed Wing failed!")

async def transition_mc(drone):
    print("Transitioning to Multi Copter...")
    try:
        await drone.action.transition_to_multicopter()
    except:
        print("ERROR: Transitioning to Multi Copter failed!")

async def return_drone(drone):
    print("Returning to home...")
    try:
        await drone.action.return_to_launch()
        
    except:
        print("ERROR: Return command failed!")


async def land_drone(drone):
    print("Landing the drone...")
    try:
        await drone.action.land()        
    except:
        print("ERROR: Landing command failed!")


async def go_to(drone):
    print("Fetching amsl altitude at home location....")
    goto_loc = await asyncio.get_event_loop().run_in_executor(None, input, 'n. North\ns. South\nw. West\n')
    if goto_loc == 'n':
        lat = 35.039041
        lon = 33.345370
    elif goto_loc == 's':
        lat = 35.036830
        lon = 33.345979
    elif goto_loc == 'w':
        lat = 35.037704
        lon = 33.344649
    else:
        return        
    try:
        async for terrain_info in drone.telemetry.home():
            absolute_altitude = terrain_info.absolute_altitude_m
            break
        # To fly drone 20m above the ground plane
        flying_alt = absolute_altitude + 20.0
        # goto_location() takes AMSL altitude
        await drone.action.goto_location(lat, lon, flying_alt, 120)
    except:
        print("ERROR: Go To command failed!")


async def reboot_drone(drone):
    print("Rebooting drone....")
    try:
        await drone.action.reboot()
    except:
        print("ERROR: Reboot failed!")



async def print_status_text(drone):
    print("UUU")
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
        # async for gps_info in drone.telemetry.gps_info():
        #     print(f"GPS info: {gps_info}")          
    except asyncio.CancelledError:
        return
    

async def print_altitude(drone):
    print("AAAA")
    """ Prints the altitude when it changes """

    previous_altitude = None

    async for position in drone.telemetry.position():
        altitude = round(position.relative_altitude_m)
        if altitude != previous_altitude:
            previous_altitude = altitude
            print(f"Altitude: {altitude}")


async def read_user_input(drone):
    functions = [
        {"name": "Arm", "function": arm_drone, "obj": drone},
        {"name": "Disarm", "function": disarm_drone, "obj": drone},
        {"name": "Takeoff", "function": takeoff_drone, "obj": drone},
        {"name": "Transition FW", "function": transition_fw, "obj": drone},
        {"name": "Transition MC", "function": transition_mc, "obj": drone},
        {"name": "Go To", "function": go_to, "obj": drone},
        {"name": "Return to home", "function": return_drone, "obj": drone},
        {"name": "Land", "function": land_drone, "obj": drone},
        {"name": "Reboot", "function": reboot_drone, "obj": drone}
        # set_current_speed(speed_m_s)
        # hold()
    ]

    while True:
        if not readingSecondaryInput:
            prompt = "--------- Enter a command ---------\n"
            for i, func in enumerate(functions, start=1):
                prompt += f"{i}. {func['name']}\n"
            prompt += "----------------------------------\n"
            print(prompt)
            user_input = await asyncio.get_event_loop().run_in_executor(None, input, ' ')
            try:
                option_index = int(user_input) - 1
                selected_function = functions[option_index]["function"]
                selected_object = functions[option_index]["obj"]
                await selected_function(selected_object)
            except (ValueError, IndexError):
                print("Invalid command. Try again.")   

            # print(f"You entered: {user_input}")



async def main():

    drone = System()
    # await drone.connect(system_address="udp://:14550")
    await drone.connect(system_address="udp://:14445")

    status_text_task = asyncio.ensure_future(print_status_text(drone))
    # print_altitude_task = asyncio.ensure_future(print_altitude(drone))
    task_input = asyncio.create_task(read_user_input(drone))

    # gather all the tasks and wait for them to complete
    # await asyncio.gather(status_text_task, print_altitude_task, task_input)

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break


    # functions = [
    #     {"name": "Arm", "function": arm_drone, "obj": drone},
    #     {"name": "Disarm", "function": disarm_drone, "obj": drone},
    #     {"name": "Takeoff", "function": takeoff_drone, "obj": drone},
    #     {"name": "Transition FW", "function": transition_fw, "obj": drone},
    #     {"name": "Transition MC", "function": transition_mc, "obj": drone},
    #     {"name": "Go To", "function": go_to, "obj": drone},
    #     {"name": "Return to home", "function": return_drone, "obj": drone},
    #     {"name": "Land", "function": land_drone, "obj": drone},
    #     {"name": "Reboot", "function": reboot_drone, "obj": drone}
    # ]


    while True:
        # print("...")
        await asyncio.sleep(1)

    #     # Generate the prompt string dynamically
    #     prompt = "Enter a command:\n"
    #     for i, func in enumerate(functions, start=1):
    #         prompt += f"{i}. {func['name']}\n"
    #     prompt += "q. Quit\n"

    #     user_input = input(prompt)

    #     if user_input.lower() == 'q':
    #         break

    #     try:
    #         option_index = int(user_input) - 1
    #         selected_function = functions[option_index]["function"]
    #         selected_object = functions[option_index]["obj"]
    #         await selected_function(selected_object)
    #     except (ValueError, IndexError):
    #         print("Invalid command. Try again.")    

    # while True:
    #     user_input = input("Enter a command:\n1. Arm\n2. Disarm\n3. Takeoff\n4. Transition FW\n5. Transition MC\n6. Go To\n7. Return to home\n8. Land\nq. Quit\n")

    #     if user_input == '1':
    #         await arm_drone(drone)
    #     elif user_input == '2':
    #         await disarm_drone(drone)            
    #     elif user_input == '3':
    #         take_off_altitude = input("Enter altitude in meters: ")
    #         await takeoff_drone(drone, int(take_off_altitude))
    #     elif user_input == '4':
    #         await transition_fw(drone)
    #     elif user_input == '5':
    #         await transition_mc(drone)
    #     elif user_input == '6':
    #         await go_to(drone)               
    #     elif user_input == '7':
    #         await return_drone(drone)                
    #     elif user_input == '8':
    #         await land_drone(drone)              
    #     elif user_input.lower() == 'q':
    #         break
    #     else:
    #         print("Invalid command. Try again.")

asyncio.run(main())