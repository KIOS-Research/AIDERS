#!/bin/bash

current_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# run the monitoring app
echo -e "\n\U0001F5A5  Running monitoring app...\n"
chmod +x ${current_dir}/../_monitor/main.py
python3 ${current_dir}/../_monitor/main.py &