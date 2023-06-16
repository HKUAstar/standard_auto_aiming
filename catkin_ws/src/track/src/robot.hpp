#ifndef TRACK_ROBOT_HPP_
#define TRACK_ROBOT_HPP_

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

class Controller
{

public:
    Controller();
    void moveGimbal(double yaw_angle, double pitch_angle, double pitch_offset = 0.000, bool absolute = false);
    void shoot(int mode, int number = 1);
    void endshoot();
    bool empty();

private:
    ros::Publisher pub;
    ros::ServiceClient client;
    int count;
};
#endif
