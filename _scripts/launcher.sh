#!/bin/bash

current_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
venv_dir="${current_dir}/../venv"

# ignore files from git
git update-index --assume-unchanged ${current_dir}/../_simulator/.env
git update-index --assume-unchanged ${current_dir}/../_simulator/logs/app_logs.txt 
git update-index --assume-unchanged ${current_dir}/../db_states.txt  


# Check if virtual environment exists
if [ ! -f "$venv_dir/bin/activate" ]; then
    echo "Error: Virtual environment creation failed or is incomplete."
    echo "Recreating virtual environment..."
    rm -rf "$venv_dir"
    python3 -m venv "$venv_dir"
fi

# Activate virtual environment and install required packages
source "$venv_dir/bin/activate"

# Install required packages if not already installed
if ! "$venv_dir/bin/python" -m pip install --quiet netifaces python-dotenv requests pytz matplotlib; then
    py_minor="$($venv_dir/bin/python -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
    echo "Error: Failed to install Python dependencies in the virtual environment."
    echo "Hint: netifaces needs Python development headers and a compiler."
    echo "On Ubuntu/Debian: sudo apt install python${py_minor}-dev build-essential"
    echo "Fallback: sudo apt install python3-dev build-essential"
    exit 1
fi

# run the launcher
echo -e "\n\U1F680 Running the launcher...\n"
chmod +x ${current_dir}/../_launcher/main.py
"$venv_dir/bin/python" ${current_dir}/../_launcher/main.py &