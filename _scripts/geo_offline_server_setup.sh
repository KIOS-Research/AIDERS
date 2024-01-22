#!/bin/bash

# get the current directory
current_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

COUNTRY="cyprus" # Default country
CUSTOM_COUNTRY=""

# declare the colours
RED='\033[0;31m'

while getopts "c:" opt; do
    case "$opt" in
    c)
        CUSTOM_COUNTRY="$OPTARG"
        ;;
    \?)
        echo "Usage: $0 [-c COUNTRY]"
        exit 1
        ;;
    esac
done

if [ -n "$CUSTOM_COUNTRY" ]; then
    COUNTRY="$CUSTOM_COUNTRY"
fi

PBF_URL="https://download.geofabrik.de/europe/${COUNTRY}-latest.osm.pbf"
POLY_URL="https://download.geofabrik.de/europe/${COUNTRY}.poly"

# Function to check if a URL exists
url_exists() {
    if curl -s --head "$1" | head -n 1 | grep "200 OK" >/dev/null; then
        return 0 # URL exists
    else
        return 1 # URL does not exist
    fi
}

# Check if the PBF_URL and POLY_URL exist
if url_exists "$PBF_URL" && url_exists "$POLY_URL"; then
    echo "Using custom country: $COUNTRY"
else
    echo "Custom country '$COUNTRY' not found. Using default country: cyprus"
    COUNTRY="cyprus"
    PBF_URL="https://download.geofabrik.de/europe/cyprus-latest.osm.pbf"
    POLY_URL="https://download.geofabrik.de/europe/cyprus.poly"
fi
docker run \
    -e UPDATES=enabled \
    -e DOWNLOAD_PBF="$PBF_URL" \
    -e DOWNLOAD_POLY="$POLY_URL" \
    -v "$current_dir/../geo/osm-data:/data/database/" \
    overv/openstreetmap-tile-server:latest \
    import
