#include <iostream>
#include <roborts_msgs/GimbalAngle.h>
#include <roborts_msgs/ShootCmd.h>
#include <roborts_msgs/FricWhl.h>
//#include <number_recognition/img_msg.h>
#include "tools.h"

Controller::Controller()
{
    ros::NodeHandle n;
    pub = n.advertise<roborts_msgs::GimbalAngle>("/cmd_gimbal_angle", 10);
    client = n.serviceClient<roborts_msgs::ShootCmd>("/cmd_shoot");
    wheel_client = n.serviceClient<roborts_msgs::FricWhl>("/cmd_fric_wheel");

    /*
    roborts_msgs::FricWhl srv;
    srv.request.open = true;
    if (wheel_client.call(srv))
        std::cout << "Wheel service call successful." << std::endl;
    else
        std::cout << "Wheel service call unsuccessful." << std::endl;
    */

    count = 0;
}

void Controller::shoot(int mode, int number)
{
    roborts_msgs::ShootCmd srv;
    srv.request.mode = mode;
    srv.request.number = number;
    count++;
    if (client.call(srv))
        std::cout << "Shoot service call successful." << std::endl;
    else
        std::cout << "Shoot service call unsuccessful." << std::endl;
}

void Controller::endshoot()
{
    roborts_msgs::ShootCmd srv;
    srv.request.mode = 0;
    srv.request.number = 0;
    if (client.call(srv))
        std::cout << "End shoot service call successful." << std::endl;
    else
        std::cout << "Shoot service call unsuccessful." << std::endl;
}

void Controller::moveGimbal(double yaw_angle, double pitch_angle, double pitch_offset, bool absolute)
{
    roborts_msgs::GimbalAngle msg;
    if (absolute)
        msg.yaw_mode = 0;
    else
        msg.yaw_mode = 1;
    if (absolute)
        msg.pitch_mode = 0;
    else
        msg.pitch_mode = 1;
    msg.yaw_angle = yaw_angle;
    msg.pitch_angle = pitch_angle - pitch_offset;
    std::cout << "Move gimbal: " << yaw_angle << " " << pitch_angle << std::endl;
    pub.publish(msg);
}
