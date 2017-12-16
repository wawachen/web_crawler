/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include <vector>
#include <string>
#include "acs/Goal.h"

namespace tsc_acs {
namespace acs {

class GpsPathPoint
{
public:
  GpsPathPoint() {}
  GpsPathPoint(double lat, double lng, double heading, const GpsPathPoint & origin);
  void Set(double lat, double lng, double heading, double x, double y, double theta);

  void Get(double & x, double & y, double & theta) const
  {
    x = this->x;
    y = this->y;
    theta = this->theta;
  }

  void Get(::acs::Goal & goal) const
  {
    goal.pose.pose.x = this->x;
		goal.pose.pose.y = this->y;
		goal.pose.pose.theta = this->theta;
		goal.curvature = this->curvature;
		goal.max_speed = this->maxSpeed;
  }

  void ToXY(double lat, double lng, double heading, double &x, double &y, double &theta) const;
  void ToLatLng(double x, double y, double theta, double &lat, double &lng, double &heading) const;

  double Latitude() const {return lat;}
  double Longitude() const {return lng;}
  double Heading() const {return heading;}

  double X() const {return x;}
  double Y() const {return y;}
  double Theta() const {return theta;}

	double Curvature() const { return curvature; }
	double MaxSpeed() const { return maxSpeed; }

	void SetCurvature(double K) { curvature = K; }
	void SetMaxSpeed(double v) { maxSpeed = v; }

private:
  double lat;
  double lng;
  double heading;

  double x;
  double y;
  double theta;

	double curvature;
	double maxSpeed;
};

class GpsPath : public std::vector<GpsPathPoint>
{
public:
  GpsPath() {}
  GpsPath(const std::string & pathfile);
  void ReadFile(const std::string & pathfile, double originLat = -1000, double originLng = -1000);
  const GpsPathPoint & Origin() const {return origin;}
  GpsPathPoint & Origin() {return origin;}

  void Add(double lat, double lng, double heading);
	void CalculateMaxSpeed(double fullSpeed, double lowSpeed, double fullSpeedCurvature,
		double lowSpeedCurvature, double deceleration);
private:
	void CalculateCurvature(bool loop);
	static double CurvatureSpeed(double curvature, double fullSpeed, double lowSpeed, double fullSpeedCurvature,
		double lowSpeedCurvature);

private:
  GpsPathPoint origin;
};

}
}
