/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include <iterator>
#include "../include/acs/GpsPath.h"
#include <math.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <cerrno>
#include <cstring>

namespace tsc_acs {
namespace acs {

GpsPathPoint::GpsPathPoint(double lat, double lng, double heading, const GpsPathPoint & origin)
  : lat(lat), lng(lng), heading(heading), maxSpeed(2)
{
  origin.ToXY(lat, lng, heading, x, y, theta);
}

void GpsPathPoint::Set(double lat, double lng, double heading, double x, double y, double theta)
{
  this->lat = lat;
  this->lng = lng;
  this->heading = heading;
  this->x = x;
  this->y = y;
  this->theta = theta;
}

static const double earthRadius = 6371000;    // m

void GpsPathPoint::ToXY(double lat, double lng, double heading,
                        double &x, double &y, double &theta) const
{
  x = this->x + earthRadius * cos(this->lat * M_PI / 180) * sin((lng - this->lng) * M_PI / 180);
  y = this->y + earthRadius * sin((lat - this->lat) * M_PI / 180);
  theta = this->theta + M_PI/2 - heading;
}
void GpsPathPoint::ToLatLng(double x, double y, double theta,
                            double &lat, double &lng, double &heading) const
{
  lat = asin( (y - this->y) / earthRadius) * 180 / M_PI + this->lat;
  lng = asin( (x - this->x) / (earthRadius * cos(this->lat * M_PI / 180))) * 180 / M_PI + this->lng;
  heading = M_PI/2 - theta;
}


GpsPath::GpsPath(const std::string & pathfile)
{
  ReadFile(pathfile);
}

void GpsPath::ReadFile(const std::string & pathfile, double originLat, double originLng)
{
  std::ifstream file;
  file.open(pathfile.c_str());

  if (file.fail())
  {
    std::cerr << pathfile << "\n";
    std::cerr << std::strerror(errno) << "\t" << errno << "\n";
  }

  bool originFound = false;

	if (originLat > -360)
	{
		originFound = true;
		origin.Set(originLat, originLng, 0, 0, 0, 0);
	}

  int iLine = 0;
  std::string line;
  std::string delimiter = ",";
  while(std::getline(file, line))
  {
    size_t pos1 = line.find(delimiter);
    size_t pos2 = line.find(delimiter, pos1+1);

    if (pos1 != std::string::npos && pos2 != std::string::npos)
    {
      double lat = atof(line.substr(0, pos1).c_str());
      double lng = atof(line.substr(pos1+1, pos2-pos1-1).c_str());
      double heading = atof(line.substr(pos2+1).c_str());
      if(!originFound)
      {
        std::cout << "Origin: " << lat << ", " << lng << "\n";
        origin.Set(lat, lng, heading, 0, 0, 0);
        originFound = true;
      }
      Add(lat, lng, heading);
    }

    iLine++;
  }

	CalculateCurvature(true);
}

void GpsPath::Add(double lat, double lng, double heading)
{
  push_back(GpsPathPoint(lat, lng, heading, origin));
}

void GpsPath::CalculateCurvature(bool loop)
{
	GpsPathPoint * previous = loop ? &back() : &front();

	for (iterator i = begin(); i != end(); i++)
	{
		iterator iNext = i; iNext++;
		GpsPathPoint & next = (iNext == end()) ? (loop ? front() : back())
			: *iNext;

		double s2 = (next.X() - previous->X()) * (next.X()-previous->X())
			+ (next.Y() - previous->Y()) * (next.Y() - previous->Y());

		i->SetCurvature((s2==0) ? 0 : (fmod(next.Theta() - previous->Theta() + M_PI, 2*M_PI)-M_PI) / sqrt(s2));
		previous = &(*i);
	}
}

void GpsPath::CalculateMaxSpeed(double fullSpeed, double lowSpeed, double fullSpeedCurvature,
																double lowSpeedCurvature, double deceleration)
{
	reverse_iterator it = rbegin();
	it->SetMaxSpeed(CurvatureSpeed(it->Curvature(), fullSpeed, lowSpeed, fullSpeedCurvature, lowSpeedCurvature));
	reverse_iterator previous = it;
	it++;
	for (; it != rend(); it++)
	{
		// Maximum speed for the curvature
		double curvatureSpeed = CurvatureSpeed(it->Curvature(), fullSpeed, lowSpeed, fullSpeedCurvature, lowSpeedCurvature);
		double s = sqrt((it->X() - previous->X())*(it->X() - previous->X()) +
									(it->Y() - previous->Y())*(it->Y() - previous->Y()));
		// Maximum speed to decelerate for future speed
		double decelerationSpeed = sqrt(2 * deceleration * s + previous->MaxSpeed() * previous->MaxSpeed());

		it->SetMaxSpeed(std::min(curvatureSpeed, decelerationSpeed));

		previous = it;
	}
}

double GpsPath::CurvatureSpeed(double curvature, double fullSpeed, double lowSpeed,
	double fullSpeedCurvature, double lowSpeedCurvature)
{
	double K = fabs(curvature);
	if (K < fullSpeedCurvature)
	{
		return fullSpeed;
	}
	else if (K > lowSpeedCurvature)
	{
		return lowSpeed;
	}
	else
	{
		return fullSpeed - (fullSpeed - lowSpeed) * (K - fullSpeedCurvature) / (lowSpeedCurvature - fullSpeedCurvature);
	}
}

}
}
