/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "BigEndian.h"
#include "ibeo/FusionNode.h"
#include "ibeo/FusionObjectList.h"

namespace tsc_acs {
namespace ibeo {

void FusionNode::handleFusionObject(const ::ibeo::Message & msg)
{
    ::ibeo::FusionObjectList list;

    list.midscan_timestamp = timeFromBigEndianNTP64(&msg.data[0]);

    uint16_t nObjects = fromBigEndian<uint16_t>(&msg.data[8]);
    list.objects.resize(nObjects);

    uint32_t offset = 10;
    bool error = false;
    for(uint16_t iObject=0; iObject<nObjects; iObject++)
    {
        if (offset + 131 > msg.data.size())
        {
            error = true;
            ROS_ERROR("fusion object message to small for expected content: A");
            break;
        }

        const uint8_t * objectData = &msg.data[offset];
        ::ibeo::FusionObject & object = list.objects[iObject];

        object.object_id = fromBigEndian<uint16_t>(&objectData[0]);
        object.flags = fromBigEndian<uint16_t>(&objectData[2]);
        object.age = fromBigEndian<uint32_t>(&objectData[4]);
        object.timestamp = timeFromBigEndianNTP64(&objectData[8]);
        object.prediction_age = fromBigEndian<uint16_t>(&objectData[16]);
        object.classification = objectData[18];
        object.classification_quality = objectData[19];
        object.classification_age = fromBigEndian<uint32_t>(&objectData[20]);

        readPoint2D(object.center, &objectData[40]);
        readPoint2D(object.center_sigma, &objectData[48]);
        readPoint2D(object.size, &objectData[56]);

        object.course_angle = floatFromBigEndian(&objectData[72]);
        object.course_angle_sigma = floatFromBigEndian(&objectData[76]);

        readPoint2D(object.relative_velocity, &objectData[80]);
        readPoint2D(object.relative_velocity_sigma, &objectData[88]);
        readPoint2D(object.absolute_velocity, &objectData[96]);
        readPoint2D(object.absolute_velocity_sigma, &objectData[104]);

        uint8_t nContourPoints = objectData[130];
        if (offset + 168 + 8*nContourPoints > msg.data.size())
        {
            error = true;
            ROS_ERROR("fusion object message to small for expected content: B");
            break;
        }

        object.index_closest_point = objectData[131];
        object.reference_point_location = fromBigEndian<uint16_t>(&objectData[132]);
        readPoint2D(object.reference_point, &objectData[134]);
        readPoint2D(object.reference_point_sigma, &objectData[142]);
        object.reference_point_correlation = floatFromBigEndian(&objectData[150]);
        object.priority = fromBigEndian<uint16_t>(&objectData[162]);
        object.existence_measurement = floatFromBigEndian(&objectData[164]);

        object.contour.resize(nContourPoints);
        for(uint8_t iContour=0; iContour<nContourPoints; iContour++)
            readPoint2D(object.contour[iContour], &objectData[168+8*iContour]);

        offset += 168 + 8*nContourPoints;
    }


    if (!error)
    {
        ROS_ASSERT(offset == msg.data.size());
        pubFusionObject.publish(list);
    }
}

void FusionNode::readPoint2D(::ibeo::Point2D32 & point, const uint8_t * data)
{
    point.x = floatFromBigEndian(&data[0]);
    point.y = floatFromBigEndian(&data[4]);
}


}
}

