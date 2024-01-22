#!/bin/bash

RED='\033[0;31m'
NC='\033[0m'  # No Color

# Check if the script is called with the "noinput" parameter
if [ "$1" == "noinput" ]; then
    answer="y"
else
    echo -e "${RED}\n\U0001F6A9 This script will delete ALL the data related to Docker! \U0001F6A9\n${NC}"
    read -p "Are you sure you want to proceed? (y/n): " answer
fi


if [[ "$answer" =~ [Yy] ]]; then
    docker stop $(docker ps -aq)
    docker rm $(docker ps -aq)
    docker rmi -f $(docker images -aq)
    docker builder prune -a -f
    docker network prune -f
    docker volume prune -f
    echo -e "\nDocker data deleted!\n"
else
    echo -e "\nAborted.\n"
    exit 1
fi
