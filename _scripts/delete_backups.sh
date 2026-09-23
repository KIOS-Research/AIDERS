#!/bin/bash

# Determine the current directory of the script
CURRENT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Define the path to the _backups folder
backups_folder="$CURRENT_DIR/../_backups"

# Check if the _backups folder exists
if [ -d "$backups_folder" ]; then
  # Delete all files in the _backups folder
  rm -f "$backups_folder"/*
  echo "All files in the _backups folder have been deleted."
else
  echo "The _backups folder does not exist."
fi
