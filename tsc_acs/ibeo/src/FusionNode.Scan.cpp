/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "BigEndian.h"
#include "ibeo/FusionNode.h"
#include "ibeo/SimplePointCloud.h"

namespace tsc_acs {
namespace ibeo {

void FusionNode::handleFusionScan_PointCloud(const ::ibeo::Message & msg)
{
    uint16_t scanPoints = fromBigEndian<uint16_t>(&msg.data[18]);
    uint8_t scanners = msg.data[20];
    SimplePointCloud<float, sensor_msgs::PointField::FLOAT32> cloud(scanPoints, "map");

    for (int iPoint=0; iPoint < scanPoints; iPoint++)
    {
        const uint8_t * pScanPoint = &msg.data[24 + scanners*148 + iPoint*28];
        cloud.SetPoint(iPoint,
                floatFromBigEndian(&pScanPoint[0]),
                floatFromBigEndian(&pScanPoint[4]),
                floatFromBigEndian(&pScanPoint[8]));
    }

    pubFusionPointCloud.publish(cloud.GetPointCloud());
}

void FusionNode::handleFusionScan(const ::ibeo::Message & msg)
{
    ::ibeo::FusionScan scan;

    scan.start_time = timeFromBigEndianNTP64(&msg.data[0]);
    scan.end_time_offset = durationFromBigEndian_us(&msg.data[8]);
    scan.flags = fromBigEndian<uint32_t>(&msg.data[12]);
    scan.scan_number = fromBigEndian<uint16_t>(&msg.data[16]);

    uint16_t scanPoints = fromBigEndian<uint16_t>(&msg.data[18]);
    uint8_t scanners = msg.data[20];

    ROS_ASSERT(msg.data.size() == 24 + scanners*148 + scanPoints*28);

    for (int iScanner=0; iScanner < scanners; iScanner++)
    {
        ::ibeo::ScannerInfo scannerInfo;
        readScannerInfo(scannerInfo, &msg.data[24 + iScanner*148]);
        scan.scanner_info.push_back(scannerInfo);
    }

    for (int iScan=0; iScan < scanPoints; iScan++)
    {
        ::ibeo::ScanPoint scanPoint;
        readScanPoint(scanPoint, &msg.data[24 + scanners*148 + iScan*28]);
        scan.scan_points.push_back(scanPoint);
    }

    pubFusionScan.publish(scan);
}

void FusionNode::readScannerInfo(::ibeo::ScannerInfo & scannerInfo, const uint8_t * data)
{
    scannerInfo.device_id = data[0];
    scannerInfo.scanner_type = data[1];
    scannerInfo.scan_number = fromBigEndian<uint16_t>(&data[2]);
    scannerInfo.start_angle = floatFromBigEndian(&data[8]);
    scannerInfo.end_angle = floatFromBigEndian(&data[12]);
    scannerInfo.start_time = timeFromBigEndianNTP64(&data[16]);
    scannerInfo.end_time = timeFromBigEndianNTP64(&data[24]);
    scannerInfo.start_time_device = timeFromBigEndianNTP64(&data[32]);
    scannerInfo.end_time_device = timeFromBigEndianNTP64(&data[40]);
    scannerInfo.scan_frequency = floatFromBigEndian(&data[48]);
    scannerInfo.beam_tilt = floatFromBigEndian(&data[52]);
    scannerInfo.scan_flags = fromBigEndian<uint32_t>(&data[56]);
    scannerInfo.yaw = floatFromBigEndian(&data[60]);
    scannerInfo.pitch = floatFromBigEndian(&data[64]);
    scannerInfo.roll = floatFromBigEndian(&data[68]);
    scannerInfo.offset.x = floatFromBigEndian(&data[72]);
    scannerInfo.offset.y = floatFromBigEndian(&data[76]);
    scannerInfo.offset.z = floatFromBigEndian(&data[80]);
    for(int iResolution=0; iResolution<8; iResolution++)
        readResolution(scannerInfo.resolution[iResolution], &data[84 + 8*iResolution]);
}

void FusionNode::readResolution(::ibeo::Resolution & resolution, const uint8_t * data)
{
    resolution.start = floatFromBigEndian(&data[0]);
    resolution.resolution = floatFromBigEndian(&data[4]);
}

void FusionNode::readScanPoint(::ibeo::ScanPoint & scanPoint, const uint8_t * data)
{
    scanPoint.point.x = floatFromBigEndian(&data[0]);
    scanPoint.point.y = floatFromBigEndian(&data[4]);
    scanPoint.point.z = floatFromBigEndian(&data[8]);
    scanPoint.echo_width = floatFromBigEndian(&data[12]);
    scanPoint.device_id = data[16];
    scanPoint.layer = data[17];
    scanPoint.echo = data[18];
    scanPoint.timestamp = durationFromBigEndian_us(&data[20]);
    scanPoint.flags = fromBigEndian<uint16_t>(&data[24]);
}

}
}
