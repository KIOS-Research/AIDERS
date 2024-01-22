#!/bin/bash

filename='rosbagPID.txt'
n=1
#rosnode kill --all
while read line; do
# reading each line
echo "Killing PID $n : $line"
pkill -P $line
n=$((n+1))
done < $filename
rm $filename
