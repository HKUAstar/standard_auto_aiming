#include <iostream>
#include <geometry_msgs/Vector3.h>
#include <roborts_msgs/GimbalAngle.h>
#include <roborts_msgs/ShootCmd.h>
#include <ros/ros.h>
//#include <number_recognition/img_msg.h>
#include "robot.hpp"
#include <cmath>

const int H = 1080;
const int W = 1440;
const int CX = W/2;

float k = 0.02/CX;

Controller *c;

void callback(const geometry_msgs::Vector3::ConstPtr& msg){
    int x = msg->x, y = msg->y;
    ROS_INFO("Get x: %d, y:%d", x, y);
    int dx = CX-x;
    float speed = k*dx;
    c->moveGimbal(speed, 0);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "simple");
    auto nh = new ros::NodeHandle();
    ros::Subscriber armors_sub_ = nh->subscribe("/pixel", 1000, callback);
    c = new Controller();

    ros::spin();

    return 0;
}