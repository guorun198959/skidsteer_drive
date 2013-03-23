#ifndef __SMP_SKIDSTEER_CONTROLLER__H__
#define __SMP_SKIDSTEER_CONTROLLER__H__

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Twist.h"
#include <string.h>

namespace skidsteer_drive
{

    class skid_drive
    {
        public:
            skid_drive(ros::NodeHandle &n);
            ~skid_drive();
            void setWheelBase(double diameter);
            void setRadius(double base);
            double getWheelBase();
            double getWheelDiameter();
            void callback(const geometry_msgs::Twist::ConstPtr& msg);

        private:
            ros::NodeHandle n;
            ros::Publisher jointPub; 
            ros::Subscriber sub;
            double wheelBase;
            double wheelRadius;
	    std::string frontLeft;
	    std::string frontRight;
	    std::string backLeft;
	    std::string backRight;
            void inverseSkidDriveKinematics(const double &linear_velocity, const double &angular_velocity); 

    };

};

#endif
