#!/bin/bash
# Wrapper script to run ground vehicle simulator inside the web Docker container

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Starting Drone RID Simulator..."
echo "Press Ctrl+C to stop"
echo ""

# Run the script inside the container
docker exec -it web bash -c "
cd /app && 
python generate_drid_drones.py
"
