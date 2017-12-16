#!/bin/bash

# Get script location
SCRIPTPATH="$(dirname "$(readlink -f "$0")")"

# Check path file exists
. $SCRIPTPATH/auxFiles/checkPathFileExists.sh $1

# Start Can
. $SCRIPTPATH/auxFiles/startCan.sh

roslaunch $SCRIPTPATH/rviz_launcher.launch & 

sleep 5

roslaunch $SCRIPTPATH/all.launch pathFileName:=$SCRIPTPATH/../routes/$PATHFILE

