// #include "math/kinematics.h"
#include "mathmatics/c++/include/differential_kinematics.hpp"

namespace amr
{
namespace differential
{

Kinematics::Kinematics(double wheel_radius,double wheel_distance, double gear_ratio) : 
  wheel_radius(wheel_radius), wheel_distance(wheel_distance),gear_ratio(gear_ratio)
{

}

Kinematics::~Kinematics()
{

}

Kinematics::Encoder()
{
  RPM_Value_l = (EncoderSpeedCounter1*(60*Control_cycle))/(Encoder_resolution*Encoder_chanel);
  EncoderSpeedCounter1 = 0;
  RPM_Value_r = (EncoderSpeedCounter2*(60*Control_cycle))/(Encoder_resolution*Encoder_chanel);
  EncoderSpeedCounter2 = 0;
}

std::pair<motor::left,motor::right> Kinematics::VWtoRPM(geometry_msgs::Twist vel)
{
  std::pair<motor::left,motor::right> rpm;
  rpm.first = ((60/(2*M_PI*wheel_radius)) * (vel.linear.x - (wheel_distance/2)*vel.angular.z) * gear_ratio);
  rpm.second = ((60/(2*M_PI*wheel_radius)) * (vel.linear.x + (wheel_distance/2)*vel.angular.z) * gear_ratio);
  return rpm;
}

geometry_msgs::Twist Kinematics::RPMtoVW(std::pair<motor::left,motor::right> rpm)
{
  geometry_msgs::Twist vel;
  std::pair<motor::left,motor::right> geared_rpm;
  geared_rpm.first = rpm.first / gear_ratio;
  geared_rpm.second = rpm.second / gear_ratio;
  vel.linear.x = (geared_rpm.second + geared_rpm.first) * (M_PI*wheel_radius/60);
  vel.angular.z = (geared_rpm.second - geared_rpm.first)*((M_PI*wheel_radius)/(30*wheel_distance));
  return vel;
}

}

namespace symmetrybicycle
{

Kinematics::Kinematics(double wheel_radius,double wheel_distance, double velocity_gear_ratio, double steering_gear_ratio) : 
  wheel_radius(wheel_radius), wheel_distance(wheel_distance),velocity_gear_ratio(velocity_gear_ratio),steering_gear_ratio(steering_gear_ratio)
{

}

Kinematics::~Kinematics()
{

}

std::pair<motor::front,motor::rear> Kinematics::toRPM(geometry_msgs::Twist vel)
{
  double v,v_x,v_y,v_th,w,R;
  double velocity, steering, a_r;
  v_x = vel.linear.x;
  v_y = vel.linear.y;
  w = vel.angular.z;

  v = std::copysign(hypot(v_y,v_x),v_x);
  v_th = atan2(v_y,v_x);
  // velocity = v * 60 / (2*M_PI*wheel_radius);
  velocity = v * 60 / (M_PI*wheel_radius);

  
  std::pair<motor::front,motor::rear> rpm;
  rpm.first = velocity;
  rpm.second = velocity;
  rpm.first *= velocity_gear_ratio;
  rpm.second *= velocity_gear_ratio;
  return rpm;
}

std::pair<motor::front_steering,motor::rear_steering> Kinematics::toSteering(geometry_msgs::Twist vel)
{
  double v,v_x,v_y,v_th,w,R;
  double velocity, steering, a_r;
  v_x = vel.linear.x;
  v_y = vel.linear.y;
  w = vel.angular.z;

  v = std::copysign(hypot(v_y,v_x),v_x);
  v_th = atan2(v_y,v_x);
  velocity = v * 60 / (2*M_PI*wheel_radius);
  // velocity = 
  if (fabs(w)<0.0001)
  {
    steering = v_th;
    if (steering > M_PI/2.)
      steering -=M_PI;
    else if (steering < -M_PI/2.)
      steering +=M_PI;
  }
  else
  {
    R = v/w;
    steering = asin(wheel_distance/(2*R));
  }
  static int cntt = 0;
  if (cntt++>10)
  {
  std::cout << "v: "<< v << "w: "<< w << " steering cal : " << steering << " diag : " << v_th << std::endl;
    cntt=0;
  }

  std::pair<motor::front_steering,motor::rear_steering> rpm;
  rpm.first = -steering;
  rpm.second = steering;
  // rpm.first *= steering_gear_ratio;
  // rpm.second *= steering_gear_ratio;
  return rpm;
}

geometry_msgs::Twist Kinematics::toVW(std::pair<motor::front,motor::rear> rpm, std::pair<motor::front_steering,motor::rear_steering> steering)
{
  double velocity, steering_angle;
  velocity = (rpm.first + rpm.second)/2;
  steering_angle =std::copysign((fabs(steering.first) + fabs(steering.second))/2.0, steering.second);

  geometry_msgs::Twist vel;
  vel.linear.x  = velocity / velocity_gear_ratio * M_PI * wheel_radius / 60;

  // vel.angular.z = 2* vel.linear.x * sin(steering_angle / steering_gear_ratio) / wheel_distance;
  vel.angular.z = 2* vel.linear.x * sin(steering_angle) / wheel_distance;
  return vel;
}

}
}