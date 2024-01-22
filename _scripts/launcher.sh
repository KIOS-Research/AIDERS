#!/bin/bash

current_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ignore files from git
git update-index --assume-unchanged ${current_dir}/../_simulator/.env
git update-index --assume-unchanged ${current_dir}/../_simulator/logs/app_logs.txt 
git update-index --assume-unchanged ${current_dir}/../db_states.txt  

# run the launcher
echo -e "\n\U1F680 Running the launcher...\n"
chmod +x ${current_dir}/../_launcher/main.py
python3 ${current_dir}/../_launcher/main.py &