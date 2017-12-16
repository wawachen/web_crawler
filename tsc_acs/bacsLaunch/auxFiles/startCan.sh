#!/bin/bash

# Initiate CAN channels
echo "Initiating can channels"

## Can channels down and then up
sudo ip link set can0 down
sudo ip link set can1 down

sudo ip link set can0 up type can bitrate 500000
sudo ip link set can1 up type can bitrate 500000

