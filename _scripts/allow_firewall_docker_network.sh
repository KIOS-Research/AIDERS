#!/bin/bash

current_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Read the subnet from config.ini
SUBNET=$(awk -F= '/^SUBNET/ {print $2}' "${current_dir}"/../config.ini)

sudo ufw allow from "${SUBNET}"