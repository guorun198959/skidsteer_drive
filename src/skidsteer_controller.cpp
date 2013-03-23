#include "skidsteer_drive/skidsteer_controller.h"

namespace skidsteer_drive
{

    skid_drive::skid_drive(ros::NodeHandle &n) : n(n)
    {
        ros::NodeHandle nh("~");
        nh.param<double>("wheel_base", wheelBase, 1.0);
        nh.param<double>("wheel_radius", wheelRadius, 0.2);
        nh.param<std::string>("front_left", frontLeft, "front_left");
        nh.param<std::string>("front_right", frontRight, "front_right");
        nh.param<std::string>("back_left", backLeft, "back_left");
        nh.param<std::string>("back_right", backRight, "back_right");

        if (wheelBase <= 0)
        {
            ROS_WARN("wheel_base must be greater than zero");
            wheelBase = 1.0;
        }
        if (wheelRadius <= 0)
        {
            ROS_WARN("wheel_radius must be greater than zero");
            wheelRadius = 0.2;
        }

        sub = n.subscribe("cmd_vel",1, &skid_drive::callback, this);


        jointPub = n.advertise<trajectory_msgs::JointTrajectory>("cmd_joint_traj", 1);

    }

    skid_drive::~skid_drive()
    {
        sub.shutdown();
        jointPub.shutdown();
    }

    void skid_drive::setWheelBase(double base)
    {
        wheelBase = base;
    }
    void skid_drive::setRadius(double radius)
    {
        wheelRadius = radius;
    }
    double skid_drive::getWheelBase()
    {
        return wheelBase;
    }
    double skid_drive::getWheelDiameter()
    {
        return wheelRadius;
    }
    void skid_drive::callback(const geometry_msgs::Twist::ConstPtr& msg)
    {

        inverseSkidDriveKinematics(msg->linear.x, msg->angular.z);
    }
    //Convert to velocity for each wheel in radians/second
    void skid_drive::inverseSkidDriveKinematics(const double &linear_velocity, const double &angular_velocity) {

        trajectory_msgs::JointTrajectory driveCtrl;
        trajectory_msgs::JointTrajectoryPoint point;

        double chL;//Left chanel velocity.
        double chR;//Right chanel Velocity.

        chR = (2*linear_velocity+angular_velocity*wheelBase)/(2*wheelRadius);
        chL = (2*linear_velocity-angular_velocity*wheelBase)/(2*wheelRadius);


	//Name the wheels and push on the velociteis for each.
        driveCtrl.joint_names.push_back(frontLeft);
        point.velocities.push_back(chL);
        driveCtrl.joint_names.push_back(frontRight);
        point.velocities.push_back(chR);
        driveCtrl.joint_names.push_back(backLeft);
        point.velocities.push_back(chL);
        driveCtrl.joint_names.push_back(backRight);
        point.velocities.push_back(chR);

	driveCtrl.points.push_back(point);

        jointPub.publish(driveCtrl);
    }
};

