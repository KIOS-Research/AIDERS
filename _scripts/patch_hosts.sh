#!/bin/bash

current_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Path to the .env file
ENV_FILE="${current_dir}/../.env"

# Path to the /etc/hosts file
HOSTS_FILE="/etc/hosts"

# Extract the IP address from the .env file
NET_IP=$(grep -E '^NET_IP=' "$ENV_FILE" | cut -d'=' -f2)

# Check if the IP address is present
if [ -n "$NET_IP" ]; then
    # Add the entry to the /etc/hosts file
    echo "$NET_IP   aiders" | sudo tee -a "$HOSTS_FILE"
    echo "Host entry added to $HOSTS_FILE"
else
    echo "Error: NET_IP not found in $ENV_FILE"
fi
