#ifndef CLASSIC__CAMERA_CPP_
#define CLASSIC__CAMERA_CPP_

#include "ros/ros.h"
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    camera::Camera cam;
    cv::Mat img;
    image_transport::Publisher pub = it.advertise("/detection/img", 1);
    while(ros::ok()){
        cam.ReadImg(img);
        if(img.empty()) continue;
        cv::Mat resized_image;
        cv::Size new_size(640, 640); // the new size of the image
        cv::resize(img, resized_image, new_size, 0, 0, cv::INTER_LINEAR); // the resize function
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized_image).toImageMsg();
        pub.publish(msg);
        ros::spinOnce();
    }

    return 0;
}

#endif