#!/bin/bash
echo "-- Running entrypoint.drid.sh --"

# Set up D-Bus environment variable to use host's system bus
export DBUS_SYSTEM_BUS_ADDRESS=unix:path=/run/dbus/system_bus_socket

# Check D-Bus connection
echo "Checking D-Bus socket..."
ls -la /run/dbus/system_bus_socket 2>/dev/null || echo "Warning: D-Bus socket not found"

# Check for Bluetooth adapter
echo "Checking for Bluetooth adapter..."
if command -v hciconfig &> /dev/null; then
    hciconfig -a || echo "No Bluetooth adapter found or permission denied"
fi

# Check if bluetoothd is accessible
if command -v bluetoothctl &> /dev/null; then
    echo "Testing bluetoothctl..."
    timeout 2 bluetoothctl show 2>&1 || echo "bluetoothctl not responding (this is OK if using host's daemon)"
fi

# wait for mysql (optional - only if needed)
# while ! echo exit | nc -z $DB_HOST $DB_PORT > /dev/null 2>&1 ; do
# 	sleep 1
# done

# run the python application
echo -e "\n** Running Drone RID Scanner **"
python -u /app/main.py &

# start the HTTP server (if needed in future)
# echo -e "\n** Starting the HTTP server **"
# cd app && gunicorn -w 5 -b 0.0.0.0:8767 server:app &

echo "-- END entrypoint.drid.sh --"

# keep the container alive
tail -f /dev/null
