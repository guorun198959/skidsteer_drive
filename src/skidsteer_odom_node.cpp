#include "skidsteer_drive/skidsteer_odom.h"

int main(int argc, char** argv){
  
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  
  skidsteer_drive::diffdrive profile(n);

  ros::spin();
  return 0;
  }
