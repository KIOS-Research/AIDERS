#!/bin/bash

# Check if the correct number of arguments is provided
if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <droneName> <value>"
  exit 1
fi

# Set the drone name and boolean value from command-line arguments
droneName=$1
value=$2

# Set the topic name
topicName="/$droneName/simulateBatteryFailure"

# Publish the boolean message
rostopic pub $topicName std_msgs/Bool "data: $value" --once
