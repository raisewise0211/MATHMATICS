#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include <geometry_msgs/Twist.h>
#include <math.h>

namespace amr
{
namespace motor
{
typedef double left;
typedef double right;
typedef double front;
typedef double rear;
typedef double front_steering;
typedef double rear_steering;
}

namespace differential
{
class Kinematics
{
private:

  double wheel_radius;
  double wheel_distance;
  double gear_ratio;

public:
  Kinematics(double wheel_radius,double wheel_distance, double gear_ratio=1.);
  ~Kinematics();

  std::pair<motor::left,motor::right> toRPM(geometry_msgs::Twist vel);
  geometry_msgs::Twist toVW(std::pair<motor::left,motor::right> rpm);

};

}

namespace symmetrybicycle
{
class Kinematics
{
private:

  double wheel_radius;
  double wheel_distance;
  double velocity_gear_ratio;
  double steering_gear_ratio; // gear_ratio * motor pole * 3

public:
  Kinematics(double wheel_radius,double wheel_distance, double velocity_gear_ratio=1., double steering_gear_ratio=1.);
  ~Kinematics();

  std::pair<motor::front,motor::rear> toRPM(geometry_msgs::Twist vel);
  std::pair<motor::front_steering,motor::rear_steering> toSteering(geometry_msgs::Twist vel);
  geometry_msgs::Twist toVW(std::pair<motor::front,motor::rear> rpm, std::pair<motor::front_steering,motor::rear_steering> steering);

};

}

}

#endif