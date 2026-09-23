#!/bin/bash

# MAVProxy routing script for UAV simulator
# This connects to the simulator/UAV and creates multiple output ports

# Activate virtual environment if it exists
if [ -d ".venv" ]; then
    source .venv/bin/activate
    echo "Virtual environment activated"
fi

# Start MAVProxy with routing
# Adjust the master connection based on your simulator's MAVLink output

mavproxy.py \
    --master=udp:0.0.0.0:14550 \
    --out=udp:0.0.0.0:14551 \
    --out=udp:0.0.0.0:14552 \
    --aircraft=SIMULATOR \
    --cmd="set heartbeat 0" \
    --daemon

# mavproxy.py \
#     --master=udp:127.0.0.1:14550 \
#     --out=udp:127.0.0.1:14551 \
#     --out=udp:127.0.0.1:14552 \
#     --aircraft=SIMULATOR \
#     --cmd="set heartbeat 0" \
#     --daemon

echo "MAVProxy started!"
echo "Connect QGroundControl to UDP port 14551"
echo "Connect Platform to UDP port 14552"
