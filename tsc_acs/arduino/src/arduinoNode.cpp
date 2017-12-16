/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include <ros.h>
#include "std_msgs/Int16.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt32.h"

#include <Arduino.h>

#include "PPMDriver.h"
#include "PPMSingleton.h"

#include "../Arduino/libraries/ros_lib/arduino/rcReader.h"
#include "../Arduino/libraries/ros_lib/arduino/arduinoDiagnostics.h"
#include "../Arduino/libraries/ros_lib/arduino/deadMansHandleReading.h"

ros::NodeHandle nh;

arduino::rcReader rc_msg;
ros::Publisher pubRC("arduino/rc_in", &rc_msg);

arduino::deadMansHandleReading deadMansHandleInput;
ros::Publisher pubDeadMansHandle("arduino/deadMansHandle", &deadMansHandleInput);

//Diagnostics
arduino::arduinoDiagnostics diag_msg;
ros::Publisher pubDiagnostics("arduino/diagnostics", &diag_msg);

#define RC_NUM_CHANNELS  8
#define RC_CH_INPUT_PIN  3

PPM::Reader<RC_NUM_CHANNELS> ppmReader(3);  // 8 Channels, pin 3

void setup() {
	nh.initNode();

	nh.advertise(pubRC);
	nh.advertise(pubDiagnostics);
	nh.advertise(pubDeadMansHandle);
}

void loop() {
	int16_t rc_values[8];

	uint32_t age;
	uint32_t blocks;
	uint32_t count;
	uint32_t repeats;

	ppmReader.getValues(rc_values, age, blocks, count, repeats);

	rc_msg.age = age;
	diag_msg.blocks = blocks;
	diag_msg.count = count;
	diag_msg.repeats = repeats;

	rc_msg.ch1 = rc_values[0];
	rc_msg.ch2 = rc_values[1];
	rc_msg.ch3 = rc_values[2];
	rc_msg.ch4 = rc_values[3];
	rc_msg.ch5 = rc_values[4];
	rc_msg.ch6 = rc_values[5];
	rc_msg.ch7 = rc_values[6];
	rc_msg.ch8 = rc_values[7];

	deadMansHandleInput.header.stamp = nh.now();
	deadMansHandleInput.analogReading = analogRead(A0);

	pubRC.publish(&rc_msg);
	pubDiagnostics.publish(&diag_msg);
	pubDeadMansHandle.publish(&deadMansHandleInput);

	nh.spinOnce();
}
