/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/SessionControlNode.h"
#include "lutz/PodCanHeader.h"
#include "lutz/PodCanId.h"

namespace tsc_acs {
namespace lutz {

const uint32_t handshakeResponse[] = {
			0x00F00FFF,
			0x4FBF40B0,
			0x16E619E9,
			0x59A956A6,
			0x8A7A8575,
			0xC535CA3A,
			0x9C6C9363,
			0xD323DC2C,
			0x2DDD22D2,
			0x62926D9D,
			0x3BCB34C4,
			0x74847B8B,
			0xA775A858,
			0xE818E717,
			0xB141BE4E,
			0xFE0EF101
		};

typedef struct
{
    _header header;
    uint8_t ACSSeed:4;
    uint8_t ACSSessionRestartRequest:2;
    uint8_t unused:2;
    uint32_t ACSKeyResponse;
    uint8_t ACSCommsQOS;
} __attribute__((packed)) PacketACS_HANDSHAKE;

void SessionControlNode::handlePOD_HANDSHAKE(const ::lutz::Handshake & msg)
{
	handshakeMsg = msg;
    if (msg.key_response == handshakeResponse[seed])
    {
		// Correct response, increase quality of service.
		qos += 3;
		if (qos > 31)
			qos = 31;
		ROS_INFO("QOS: %d : %d", qos, msg.qos);
		seed = (seed+1) % (sizeof(handshakeResponse)/sizeof(uint32_t));
		lastRx = msg.header.stamp;

		ROS_INFO("Q:%d | id:%x", (int)msg.qos, msg.pod_header.id);
	}
	podSeed = msg.seed;
	frameCount = msg.pod_header.frame_count;
	sessionId = msg.pod_header.session_id;
}

void SessionControlNode::Handshake()
{
	// If it has been 60ms since the last correct handshake reduce quality of service.
	ros::Time now = ros::Time::now();
	if (now - lastRx > qosTimeout)
	{
		if(qos > 0)
		qos--;
		lastRx = now;
	}

	::lutz::Handshake msgOut;
	msgOut.pod_header.id = ACS_HANDSHAKE;
	msgOut.pod_header.frame_count = frameCount;
	msgOut.pod_header.session_id = sessionId;

	msgOut.seed = seed;
	msgOut.key_response = handshakeResponse[podSeed];
	msgOut.qos = qos;

	pubACS_HANDSHAKE.publish(msgOut);
}

}
}


