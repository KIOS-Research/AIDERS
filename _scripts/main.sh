#!/bin/bash

current_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# run the scripts app
echo -e "\n\U0001F5A5  Running tools app...\n"
chmod +x ${current_dir}/main.py
python3 ${current_dir}/main.py &