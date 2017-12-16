#!/bin/bash

# Get script location
SCRIPTPATH="$(dirname "$(readlink -f "$0")")"

# Check path file exists
. $SCRIPTPATH/auxFiles/checkPathFileExists.sh $1

# Mount disk
. $SCRIPTPATH/auxFiles/mountDisk.sh

# Start Can
. $SCRIPTPATH/auxFiles/startCan.sh

## Launch nodes
roslaunch $SCRIPTPATH/on_axle_stands.launch pathFileName:=$SCRIPTPATH/../routes/$PATHFILE