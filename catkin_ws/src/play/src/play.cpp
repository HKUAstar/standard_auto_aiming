#include <ros/ros.h>
#include <roborts_msgs/GimbalAngle.h>
#include "tools.h"
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "play");

    auto controller = Controller();

    // controller.moveGimbal(0.0, 0.4, 0.00, true);
    // ROS::rate time = 1;
    while (ros::ok())
    {
        controller.moveGimbal(0.00, 0.005, 0.00);
        ros::spinOnce();
    }
    
    return 0;
}
