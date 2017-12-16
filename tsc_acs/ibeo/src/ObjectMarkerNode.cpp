/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "ibeo/ObjectMarker.h"
#include <cmath>
#include "../../version/version.h"

namespace tsc_acs {
namespace ibeo {

ObjectMarkerNode::ObjectMarkerNode()
{
	ReportVersion(node);
  sub = node.subscribe("ibeo/fusion_object", 10, &ObjectMarkerNode::rxCallback, this);
  pub = node.advertise<visualization_msgs::MarkerArray>("ibeo/object_marker", 10);
}

void ObjectMarkerNode::spin()
{
  ros::spin();
}

void ObjectMarkerNode::rxCallback(const ::ibeo::FusionObjectList & msg)
{
  const float classColor[][3] = {
      {0.6f, 0.6f, 0.6f}, // 0 Unclassified
      {0.6f, 0.3f, 0.0f}, // 1 Unknown Small
      {0.6f, 0.0f, 0.0f}, // 2 Unknwon Big
      {1.0f, 1.0f, 0.1f}, // 3 Pedestrian
      {0.1f, 1.0f, 0.1f}, // 4 Bike
      {0.1f, 0.0f, 1.0f}, // 5 Car
      {0.9f, 0.1f, 1.0f}, // 6 Truck
  };
  const int classColorSize = 7;

  // if the array has grown, grow marker array
  // otherwise delete additional markers.
  // Marker array is shrunk after publication.
  if (markerArray.markers.size() < msg.objects.size()*3)
    markerArray.markers.resize(msg.objects.size()*3);
  else
    for(int i=msg.objects.size(); i<markerArray.markers.size()*3; i++)
      markerArray.markers[i].action = visualization_msgs::Marker::DELETE;

  for(int i=0; i<msg.objects.size(); i++)
  {
    const ::ibeo::FusionObject & object = msg.objects[i];

    // Object Marker
    visualization_msgs::Marker & marker = markerArray.markers[i];

    marker.header.frame_id = "map";
    marker.id = i;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = object.center.x;
    marker.pose.position.y = object.center.y;
    marker.pose.position.z = -0.05;

    marker.pose.orientation.x = std::cos(object.course_angle/2);
    marker.pose.orientation.y = std::sin(object.course_angle/2);
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;

    marker.scale.x = object.size.x;
    marker.scale.y = object.size.y;
    marker.scale.z = 0.05;

    if (object.classification < classColorSize)
    {
      marker.color.r = classColor[object.classification][0];
      marker.color.g = classColor[object.classification][1];
      marker.color.b = classColor[object.classification][2];
    }
    else
    {
      marker.color.r = 0.1f;
      marker.color.g = 0.1f;
      marker.color.b = 0.1f;
    }
    marker.color.a = 0.75;

    marker.lifetime = ros::Duration();

    // Variance Marker
    visualization_msgs::Marker & markerVariance = markerArray.markers[i + msg.objects.size()];

    markerVariance.header.frame_id = "map";
    markerVariance.id = i + msg.objects.size();
    markerVariance.type = visualization_msgs::Marker::CYLINDER;
    markerVariance.action = visualization_msgs::Marker::ADD;

    markerVariance.pose.position.x = object.center.x;
    markerVariance.pose.position.y = object.center.y;
    markerVariance.pose.position.z = 0;

    markerVariance.pose.orientation.x = 0.0;
    markerVariance.pose.orientation.y = 0.0;
    markerVariance.pose.orientation.z = 0.0;
    markerVariance.pose.orientation.w = 1.0;

    // Size based on standard deviation of centre
    const float maxSigma = 0.25f;
    markerVariance.scale.x = std::min(object.center_sigma.x, maxSigma);
    markerVariance.scale.y = std::min(object.center_sigma.y, maxSigma);
    markerVariance.scale.z = 0.01;

    markerVariance.color.r = 1.0f;
    markerVariance.color.g = 1.0f;
    markerVariance.color.b = 1.0f;
    markerVariance.color.a = (object.center_sigma.x > maxSigma || object.center_sigma.y > maxSigma) ? 0.2f : 1.0f;

    markerVariance.lifetime = ros::Duration();

    // Contour Marker
    visualization_msgs::Marker & markerContour = markerArray.markers[i + 2 * msg.objects.size()];

    markerContour.header.frame_id = "map";
    markerContour.id = i + 2 * msg.objects.size();
    markerContour.type = visualization_msgs::Marker::LINE_STRIP;
    markerContour.action = visualization_msgs::Marker::ADD;

    markerContour.pose.position.x = 0;
    markerContour.pose.position.y = 0;
    markerContour.pose.position.z = 0;

    markerContour.pose.orientation.x = 0.0;
    markerContour.pose.orientation.y = 0.0;
    markerContour.pose.orientation.z = 0.0;
    markerContour.pose.orientation.w = 1.0;

    // Line width
    markerContour.scale.x = 0.05;

    // Colour yellow if limited.
    markerContour.color.r = 1.0f;
    markerContour.color.g = 1.0f;
    markerContour.color.b = 1.0f;
    markerContour.color.a = 1.0;

    for(std::vector< ::ibeo::Point2D32>::const_iterator contourPoint = object.contour.begin();
        contourPoint != object.contour.end(); contourPoint++)
    {
      geometry_msgs::Point pnt;
      pnt.x = contourPoint->x;
      pnt.y = contourPoint->y;
      pnt.z = 0.01;
      markerContour.points.push_back(pnt);
    }

    markerContour.lifetime = ros::Duration();
  }

  pub.publish(markerArray);
  markerArray.markers.resize(msg.objects.size());
}

}
}


