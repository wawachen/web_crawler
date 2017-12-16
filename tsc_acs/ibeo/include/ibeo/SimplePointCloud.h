/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "sensor_msgs/PointCloud2.h"

namespace tsc_acs {

template<typename T, uint8_t datatype>
class SimplePointCloud
{
public:
	SimplePointCloud(uint32_t pointCount, const char * frame_id)
	{
		cloud.header.frame_id = frame_id;
		cloud.height = 1;
		cloud.is_bigendian = false;
		cloud.point_step = 3 * sizeof(T);
		cloud.is_dense = true;
		cloud.width = pointCount;
		cloud.row_step = 3 * sizeof(T) * pointCount;
		cloud.data.resize(3 * sizeof(T) * pointCount);
		cloud.header.stamp = ros::Time::now();

		sensor_msgs::PointField xField;
		xField.name = "x";
		xField.offset = 0;
		xField.datatype = datatype;
		xField.count = 1;
		cloud.fields.push_back(xField);

		sensor_msgs::PointField yField;
		yField.name = "y";
		yField.offset = 4;
		yField.datatype = datatype;
		yField.count = 1;
		cloud.fields.push_back(yField);

		sensor_msgs::PointField zField;
		zField.name = "z";
		zField.offset = 8;
		zField.datatype = datatype;
		zField.count = 1;
		cloud.fields.push_back(zField);
	}
	void SetPoint(uint32_t iPoint, T x, T y, T z)
	{
		T * data = reinterpret_cast<T*>(&cloud.data[0]);
		data[iPoint*3 + 0] = x;
		data[iPoint*3 + 1] = y;
		data[iPoint*3 + 2] = z;
	}

	const sensor_msgs::PointCloud2 & GetPointCloud() const {return cloud;}

private:
	sensor_msgs::PointCloud2 cloud;
};

}
