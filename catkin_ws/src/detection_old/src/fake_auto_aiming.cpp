#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Vector3.h>
#include <msgs/Armors.h>
#include <msgs/ArmorItem.h>
#include <number_recognition/img_msg.h>
#include "plain_detect.h"
#include "tools.h"
#include "auto_aiming.h"
using namespace cv;
using namespace std;

ros::Publisher armor_pub;

void publishArmor()
{
    //Mat cropped_img = img(Range(armor.upper_l.x, armor.lower_r.x), Range(armor.upper_l.y, armor.lower_r.y));

    msgs::Armors out_msg;
    //out_msg.header = msg->header;
    for (int i = 0; i < rand() % 4; i++)
    {
        msgs::ArmorItem item;
        if (rand() % 5 == 0)
            item.area = 23333;
        else if (rand() % 10 == 0)
            item.area = 66666;
        else
            item.area = rand() % 128000;
        if (i == 0)
            item.id = 1;
        if (i == 1)
            item.id = rand() % 2 + 3;
        if (i == 2)
            item.id = 7;
        out_msg.items.emplace_back(item);
    }
    out_msg.count = out_msg.items.size();
    armor_pub.publish(out_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_auto_aiming");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);

    srand((unsigned int)time(NULL));
    armor_pub = n.advertise<msgs::Armors>("armors", 10);
    while (ros::ok())
    {
        publishArmor();

        loop_rate.sleep();
        ros::spinOnce();
    }
}