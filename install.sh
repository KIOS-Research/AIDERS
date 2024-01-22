#!/bin/bash
#
# Script: install.sh
# Author: Michalis Demetriou
# Date: August 10, 2023
# Description: This script installs dependencies and builds the docker containers for the Aiders platform.
# Revision: 1.0
#


# get the current directory
current_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"


# read the version number from the config.ini file
ini_file="${current_dir}/config.ini"
version=$(grep 'VERSION' "$ini_file" | awk -F ' = ' '{print $2}')


# declare the colours
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
MAGENTA='\033[1;35m'
CYAN='\033[0;36m'
NC='\033[0m'


# print the intro text
clear

echo -e "${YELLOW}   _____     ___  ________    ___________ __________    _________ ${NC}"
echo -e "${RED}  /  _  \   |   | \______ \   \_   _____/ \______   \  /   _____/ ${NC}"
echo -e "${MAGENTA} /  /_\  \  |   |  |    |  \   |    __)_   |       _/  \_____  \  ${NC}"
echo -e "${CYAN}/    |    \ |   |  |    \`   \  |        \  |    |   \  /        \ ${NC}"
echo -e "${GREEN}\____|____/ |___| /_________/ /_________/  |____|___/ /_________/ ${NC}"
echo -e "${RED}                                                          v${version}${NC}"

echo -e "System Requirements:"
echo -e "    \U00002022 Debian-based Linux OS (tested on Ubuntu 22.04)"
echo -e "    \U00002022 Intel i7 CPU (10th gen. or higher)"
echo -e "    \U00002022 16Gb of RAM"
echo -e "    \U00002022 40Gb of hard disk space (SSD recommended)"
echo -e "    \U00002022 Nvidia GPU 10+ series (optional but highly recommended)"

echo -e "\nThe installation process will perform the following actions:"
echo -e "    \U00002022 install Python 3 along with some required libraries"
echo -e "    \U00002022 install Nvidia runtime libraries if an Nvidia GPU is properly configured"
echo -e "    \U00002022 build multiple Docker containers needed to run the platform"
# echo -e "    \U00002022 create a desktop shortcut for the platform's launcher"

echo -e "${YELLOW}"
echo -e "The process will be downloading big amounts of data and may take a long time depending on your hardware and the speed of your internet connection.${NC}" | fmt -s -w 80
echo -e "${NC}"

echo -e "${RED}This software is provided as-is, without any warranties of any kind.${NC}"
echo ""


# confirm before proceeding
read -p "Do you want to proceed? (y/n): " answer
if [[ "$answer" =~ [Yy] ]]; then
    echo -e "\nStarting the installation process...\n"
else
    echo -e "Installation aborted!"
    exit 0
fi


# make other bash scripts executable
chmod +x ${current_dir}/_scripts/*.sh

##########
# DOCKER #
##########


# check if docker is installed
if command -v docker &>/dev/null; then
    echo -e "✔ Docker is installed."
else
    echo -e "[ERROR]: Docker is not installed on your system. Please install it and try again."
    exit 1
fi

# check if user is in the docker group
if groups $USER | grep -q '\bdocker\b'; then
    echo -e "✔ User '$USER' is in the Docker group."
else
    echo -e "[INFO]: User '$USER' is NOT in the Docker group. Adding them now."
    sudo groupadd docker
    sudo usermod -aG docker $USER
    echo -e "[INFO]: Please log out and log back in for the changes to be applied."
    exit 1
fi


##########
# PYTHON #
##########


# Check if python3 is installed
if dpkg -s "python3" &> /dev/null; then
    echo -e "✔ Python 3 is installed."
else
    sudo apt-get install -y python3
fi

# Check if python3-tk is installed
if dpkg -s "python3-tk" &> /dev/null; then
    echo -e "✔ Python Tkinter is installed."
else
    sudo apt-get install -y python3-tk
fi

# check python dependencies for monitoring app
if dpkg -s "python3-matplotlib" &> /dev/null; then
    echo -e "✔ matplotlib is installed."
else
    sudo apt-get install -y python3-matplotlib
fi
if dpkg -s "python3-pil" &> /dev/null; then
    echo -e "✔ pillow is installed."
else
    sudo apt-get install -y python3-pil
fi
if dpkg -s "python3-pil.imagetk" &> /dev/null; then
    echo -e "✔ pillow imagetk is installed."
else
    sudo apt-get install -y python3-pil.imagetk
fi



##########
# Nvidia #
##########


# check for nvidia drivers
if lsmod | grep -qi nvidia; then
    echo "✔ Nvidia drivers are installed."
    if dpkg -l | grep -q nvidia-container-runtime; then
        echo "✔ nvidia-container-runtime is installed."
        ${current_dir}/_scripts/nvidia_container_enable.sh
        echo "nvidia-container-runtime was enabled"        
    else
        echo "✖ nvidia-container-runtime is not installed."
        # install nvidia-container-runtime
        ${current_dir}/_scripts/nvidia_container_install.sh
        echo "nvidia-container-runtime was installed"
        # enable nvidia runtime in docker-compose
        ${current_dir}/_scripts/nvidia_container_enable.sh
        echo "nvidia-container-runtime was enabled"
    fi    
else
    echo "✖ Nvidia drivers are not installed. Computer Vision will run on CPU and some features might be unavailable."
    # disable nvidia runtime in docker-compose
    ${current_dir}/_scripts/nvidia_container_disable.sh
fi

##############
# UDEV RULES #
##############


echo "Adding UDEV rules."
sudo ${current_dir}/_scripts/udev_rules_setup.sh

#########
# BUILD #
#########


# check for .env file
env_file="$current_dir/.env"

if [ -f "$env_file" ]; then
    echo -e "${RED}\nAn envirnment file already exists.\nBy proceeding all the previous data will be erased!${NC}\n"
    read -p "Do you want to proceed? (y/n): " answer
    if [[ "$answer" =~ [Yy] ]]; then
        echo -e "\nProceeding...\n"
    else
        echo -e "\nInstallation aborted!"
        exit 0
    fi
else
    # create dummy .env file
    touch "$env_file"
    echo "SQL_DATABASE=temp" >> "$env_file"
    echo "SQL_USER=installer" >> "$env_file"
    echo "SQL_PASSWORD=1234" >> "$env_file"
    echo "NET_IP=127.0.0.1" >> "$env_file"
    echo "WEB_URL=http://127.0.0.1" >> "$env_file"
    echo "NGINX_PORT=1111" >> "$env_file"
    echo "Created a new .env file."
fi

# setup offline map
if [ -z "$(ls -A "$current_dir/geo/osm-data")" ]; then
    echo "Geo Server is installing..."
    ${current_dir}/_scripts/geo_offline_server_setup.sh
else
    echo "Geo Server is already installed"
fi

# build docker images
cd $current_dir && docker compose build

#docker clean-up
docker image prune -f

# delete the database
docker rm -f db 2> /dev/null
current_dir_name=$(basename "$(dirname "${BASH_SOURCE[0]}")")
docker volume rm ${current_dir_name}_mysql-data 2> /dev/null

# delete dummy .env file
rm "$env_file"


###########
# EXTRACT #
###########

echo "Extracting large files..."
# ${current_dir}/_scripts/extract_tar_xz.sh web/django_api/aiders/static/aiders/cyprus_geolocation
${current_dir}/_scripts/extract_tar_and_merge.sh web/django_api/aiders/static/aiders/cyprus_geolocation/platform_geojson_files_buildings.geojson
${current_dir}/_scripts/extract_tar_and_merge.sh web/django_api/aiders/static/aiders/cyprus_geolocation/platform_geojson_files_roadnetwork_original.geojson
${current_dir}/_scripts/extract_tar_and_merge.sh cv/app/crowd_loc/weights/model_best.pth
${current_dir}/_scripts/extract_tar_and_merge.sh cv/app/crowd_loc/weights/model_best_half_kernels_05.pth
# SafeML
${current_dir}/_scripts/extract_tar_and_merge.sh safeml/app/safeml/29092023_Good_Results/wasserstein/train_features.npy


############
# SHORTCUT #
############


# create desktop shortcut
desktop_content="[Desktop Entry]
Version=1.0
Exec=$current_dir/_scripts/launcher.sh
Name=Aiders
GenericName=Aiders
Comment=Aiders platform launcher
Encoding=UTF-8
Terminal=false
Type=Application
Categories=Application;Network;
Icon=$current_dir/_launcher/logo-white.png"

# Write the content to the aiders.desktop file
echo "$desktop_content" > $current_dir/aiders.desktop

# copy the shortcut to the desktop
cp $current_dir/aiders.desktop ~/Desktop
echo -e "\nAiders desktop shortcut created."

# make it searchable
cp $current_dir/aiders.desktop ~/.local/share/applications/
sudo cp $current_dir/aiders.desktop /usr/share/applications/
sudo update-desktop-database

# delete shortcut from the platform's root folder
rm $current_dir/aiders.desktop


##########
# LAUNCH #
##########


echo ""
echo -e "${GREEN}Installation completed!${NC}"

# run the launcher
${current_dir}/_scripts/launcher.sh