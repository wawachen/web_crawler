#!/bin/bash

# Get script location
SCRIPTPATH="$(dirname "$(readlink -f "$0")")"

# Check path file exists
. $SCRIPTPATH/auxFiles/checkPathFileExists.sh $1

## Launch nodes
roslaunch "$SCRIPTPATH/sim_ple.launch" pathFileName:=$SCRIPTPATH/../routes/$PATHFILE
