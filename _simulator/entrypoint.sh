#!/bin/bash
echo "--- Running entrypoint.sh ---"

python -u /app/main.py

# keep the container alive
tail -f /dev/null