#include "skidsteer_drive/skidsteer_controller.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "skidsteer_controller");

    ros::NodeHandle n;


    skidsteer_drive::skid_drive drive(n);

    ros::spin();

    return 0;

}

