#!/bin/bash

# Get script location
SCRIPTPATH="$(dirname "$(readlink -f "$0")")"

# Check path file exists
. $SCRIPTPATH/auxFiles/checkPathFileExists.sh $1

# Mount disk
. $SCRIPTPATH/auxFiles/mountDisk.sh

# Start Can
. $SCRIPTPATH/auxFiles/startCan.sh


## Start rosbag and wait 3 seconds
roslaunch $SCRIPTPATH/rosbag.launch &

echo "Starting rosbag ..."
sleep 3
echo "rosbag started ..."

## Launch nodes
roslaunch $SCRIPTPATH/bacs.launch pathFileName:=$SCRIPTPATH/../routes/$PATHFILE