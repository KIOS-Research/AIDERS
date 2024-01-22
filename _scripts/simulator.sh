#!/bin/bash

current_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# run the monitoring app
echo -e "\n\U0001F6E9  Running the simulator...\n"
chmod +x ${current_dir}/../_simulator/_launcher/main.py
python3 ${current_dir}/../_simulator/_launcher/main.py &