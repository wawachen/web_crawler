#*H
#  Copyright 2017 Transport Systems Catapult (The Catapult)
#  All rights reserved
#  For use only for the purpose agreed with The Catapult
#H*

uint8 device_id

# 3 = Alasca XT
# 4 = ECU
# 5 = ibeo LUX prototype
# 6 = ibeo LUX
uint8 scanner_type

uint16 scan_number

# Radians
float32 start_angle
float32 end_angle

time start_time
time end_time
time start_time_device
time end_time_device

# Hz
float32 scan_frequency

# Radians
float32 beam_tilt

uint32 scan_flags

# mounting angles relative to vehicle
# Radians
float32 yaw
float32 pitch
float32 roll

# mounting position relative to vehicle
geometry_msgs/Point32 offset

Resolution[8] resolution