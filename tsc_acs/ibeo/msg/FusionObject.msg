#*H
#  Copyright 2017 Transport Systems Catapult (The Catapult)
#  All rights reserved
#  For use only for the purpose agreed with The Catapult
#H*

# Classification
uint8 UNCLASSIFIED = 0
uint8 UNKNOWN_SMALL = 1
uint8 UNKNOWN_BIG = 2
uint8 PEDESTRIAN = 3
uint8 BIKE = 4
uint8 CAR = 5
uint8 TRUCK = 6
uint8 UNDERDRIVABLE = 12

# Reference Point
uint16 CENTRE_OF_GRAVITY = 0
uint16 FRONT_LEFT = 1
uint16 FRONT_RIGHT = 2
uint16 REAR_RIGHT = 3
uint16 REAR_LEFT = 4
uint16 FRONT_CENTRE = 5
uint16 RIGHT_CENTRE = 6
uint16 REAR_CENTRE = 7
uint16 LEFT_CENTRE = 8
uint16 OBJECT_CENTRE = 9
uint16 UNKNOWN = 255

uint16 object_id
uint16 flags
uint32 age
time timestamp
uint16 prediction_age
uint8 classification
uint8 classification_quality
uint32 classification_age

Point2D32 center
Point2D32 center_sigma
Point2D32 size

float32 course_angle
float32 course_angle_sigma

Point2D32 relative_velocity
Point2D32 relative_velocity_sigma

Point2D32 absolute_velocity
Point2D32 absolute_velocity_sigma

uint8 index_closest_point

uint16 reference_point_location
Point2D32 reference_point
Point2D32 reference_point_sigma
float32 reference_point_correlation

uint16 priority
float32 existence_measurement

Point2D32[] contour