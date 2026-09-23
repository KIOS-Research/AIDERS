#!/bin/bash

# Define the udev rules file
udev_rules_system_file='/etc/udev/rules.d/99-aiders.rules'

# Delete old rules file
sudo rm -f $udev_rules_system_file

####ADDING UDEV RULE FOR WEATHER STATION####
rule='SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6015\", SYMLINK+=\"weatherStation\"'

sudo bash -c "echo $rule> $udev_rules_system_file"

#### LORA DEVICE VERSION 2 ################
#### ADDING UDEV RULE FOR LORA STATION ####
rule='SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"10c4\", ATTRS{idProduct}==\"ea60\", SYMLINK+=\"loraReceiver\"'

sudo bash -c "echo $rule>> $udev_rules_system_file"

#### LORA DEVICE VERSION LILYGO ################
####ADDING UDEV RULE FOR LORA STATION####
rule='SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"303a\", ATTRS{idProduct}==\"1001\", SYMLINK+=\"loraReceiver\"'

sudo bash -c "echo $rule>> $udev_rules_system_file"

# Reload udev rules
sudo udevadm control --reload-rules && sudo udevadm trigger
