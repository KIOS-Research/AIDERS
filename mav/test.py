from flask import Flask, jsonify
from mavsdk import System
import asyncio
import threading

app = Flask(__name__)
drone = System()

async def telemetry_loop():
    try:
        # Connect to the drone
        await drone.connect(system_address="udp://:14550")

        # Ensure that the drone is connected before attempting to access telemetry
        async for telemetry in drone.telemetry.position():
            print(f"GPS: {telemetry.latitude_deg}, {telemetry.longitude_deg}. ALT: {telemetry.relative_altitude_m}")
            print("---")
    except KeyboardInterrupt:
        print("Telemetry stream interrupted by user")
    finally:
        # Disconnect from the drone when the telemetry loop is interrupted
        await drone.disconnect()

def start_telemetry():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    asyncio.ensure_future(telemetry_loop())
    loop.run_forever()

@app.route('/connectToUav', methods=['POST'])
def start_telemetry_route():
    telemetry_thread = threading.Thread(target=start_telemetry)
    telemetry_thread.start()
    return jsonify({"status": "Telemetry stream started"})

def disconnect_from_drone():
    try:
        drone.action.disarm()
        drone.disconnect()
    except Exception as e:
        print(f"Error during disconnection: {e}")

@app.route('/disconnectFromUav', methods=['POST'])
def disconnect_route():
    with threading.Lock():
        threading.Thread(target=disconnect_from_drone).start()
    return jsonify({"status": "Disconnecting from drone"})

if __name__ == "__main__":
    app.run(port="8769", debug=True)
