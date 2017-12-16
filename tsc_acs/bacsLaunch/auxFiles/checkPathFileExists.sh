#!/bin/bash

# Check path file exists
if [ "$#" -ne 1 ]; then
  echo "Please provide Path filename."
  exit 1
fi

PATHFILE="$1.csv"

if [ ! -f "$SCRIPTPATH/../routes/$PATHFILE" ]; then
  echo "$SCRIPTPATH/../routes/$PATHFILE Cannot be found"
  exit 1
fi