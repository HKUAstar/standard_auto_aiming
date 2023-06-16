#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include "hikrobot_camera.hpp"

// 剪裁掉照片和雷达没有重合的视角，去除多余像素可以使rosbag包变小
// #define FIT_LIDAR_CUT_IMAGE false
// #if FIT_LIDAR_CUT_IMAGE
//     #define FIT_min_x 0
//     #define FIT_min_y 0
//     #define FIT_max_x 1440
//     #define FIT_max_y 1080
// #endif 

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    //********** variables    **********/
    cv::Mat src;
    //string src = "",image_pub = "";
    //********** rosnode init **********/
    ros::init(argc, argv, "hikrobot_camera");
    ros::NodeHandle hikrobot_camera;
    camera::Camera MVS_cap(hikrobot_camera);

    //********** rosnode init **********/
    image_transport::ImageTransport main_cam_image(hikrobot_camera);
    image_transport::CameraPublisher image_pub = main_cam_image.advertiseCamera("/hikrobot_camera/rgb", 1);

    sensor_msgs::Image image_msg;
    sensor_msgs::CameraInfo camera_info_msg;
    cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;  // 就是rgb格式 
    
    //********** 250 Hz        **********/
    // ros::Rate loop_rate(250);

      double start;
      start = static_cast<double>(cv::getTickCount());

    while (ros::ok())
    {

        // loop_rate.sleep();
        ros::spinOnce();

        MVS_cap.ReadImg(src);
        if (src.empty())
        {
            continue;
        }

        double time = ((double)cv::getTickCount() - start) / cv::getTickFrequency();
        std::cout << "Camera, Time:" << time << "\tFPS:" << 1 / time << std::endl;

        cv_ptr->image = src;
        image_msg = *(cv_ptr->toImageMsg());
        image_msg.header.stamp = ros::Time::now();  // ros发出的时间不是快门时间
        image_msg.header.frame_id = "hikrobot_camera";

        camera_info_msg.header.frame_id = image_msg.header.frame_id;
	      camera_info_msg.header.stamp = image_msg.header.stamp;
        image_pub.publish(image_msg, camera_info_msg);

        //*******************************************************************************************************************/
        //start = static_cast<double>(cv::getTickCount());
        start = (cv::getTickCount());
    }
    return 0;
}
