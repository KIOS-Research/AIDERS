#!/bin/bash

RED='\033[0;31m'
NC='\033[0m'  # No Color

# Check if the script is called with the "noinput" parameter
if [ "$1" == "noinput" ]; then
    answer="y"
else
    echo -e "${RED}\n\U0001F6A9 This script will delete ALL the media files! \U0001F6A9\n${NC}"
    read -p "Are you sure you want to proceed? (y/n): " answer
fi

current_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "$answer" =~ [Yy] ]]; then
    sudo rm -rf ${current_dir}/../web/django_api/aiders/media/*
    sudo rm -rf ${current_dir}/../web/django_api/aiders/migrations/*
    sudo rm -rf ${current_dir}/../web/django_api/results_TM/*
    echo -e "\nMedia files deleted!\n"
else
    echo -e "\nAborted.\n"
    exit 1
fi
