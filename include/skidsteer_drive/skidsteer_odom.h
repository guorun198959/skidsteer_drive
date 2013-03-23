#ifndef WHEELED_SMP_ODOM_H

#define WHEELED_SMP_ODOM_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <string.h>

namespace skidsteer_drive
{

class diffdrive {
  public:
  diffdrive(ros::NodeHandle &n);
  ~diffdrive();
  double getWheelRadius();
  double getAxleLength();
  void setWheelRadius(double radius); //in m
  void setAxleLength(double length); //in m
  
  private:
  void statesCallback(const sensor_msgs::JointState::ConstPtr& msg);
  double axleLength;
  //NOTE: Axle Length is from a wheel to the center of rotation
  //What would be considered 'regular' axle length is actually 2*axleLength
  double wheelRadius;
  ros::Publisher odom_pub;
  tf::TransformBroadcaster odom_broadcaster;
  ros::NodeHandle n;
  ros::Subscriber joint_states;
  ros::Time current_time, last_time;
  double x;
  double y;
  double th;
  std::string leftFrontJointName;
  std::string rightFrontJointName;
  std::string leftBackJointName;
  std::string rightBackJointName;
};

};

#endif
