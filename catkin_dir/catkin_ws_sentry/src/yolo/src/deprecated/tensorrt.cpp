#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "hikrobot_camera.hpp"
#include "NvInfer.h"
#include "NvOnnxParser.h"
#include "NvUffParser.h"
#include "NvInferPlugin.h"
#include "yolov8.hpp"
#include <chrono>
#include <fstream>
#include <iostream>

namespace tensorrt{

    const std::vector<std::string> CLASS_NAMES = {"0","1","2","3","4","5","6","7","9","10","11","12","13","14","15","16","17"};
    const std::vector<std::vector<unsigned int>> COLORS = {
        { 0, 114, 189 }, { 217, 83, 25 }, { 237, 177, 32 },
        { 126, 47, 142 }, { 119, 172, 48 }, { 77, 190, 238 },
        { 162, 20, 47 }, { 76, 76, 76 }, { 153, 153, 153 },
        { 255, 0, 0 }, { 255, 128, 0 }, { 191, 191, 0 },
        { 0, 255, 0 }, { 0, 0, 255 }, { 170, 0, 255 },
        { 85, 85, 0 }, { 85, 170, 0 }
    };

    auto yolov8 = new YOLOv8("/home/sentry/catkin_ws/src/yolo/src/model/best2.engine");
    cv::Mat res, image;
	cv::Size size = cv::Size{640, 640};
	int num_labels = 17;
	int topk = 100;
	float score_thres = 0.25f;
	float iou_thres = 0.65f;
    std::vector<Object> objs;

    void init(){
	    yolov8->make_pipe(true);
    }

    void infer(cv::Mat &img){
        objs.clear();
        yolov8->copy_from_Mat(img, size);
        auto start = std::chrono::system_clock::now();
        yolov8->infer();
        auto end = std::chrono::system_clock::now();
        yolov8->postprocess(objs);
        yolov8->draw_objects(img, res, objs, CLASS_NAMES, COLORS);
        auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
        printf("cost %2.4lf ms\n", tc);
    }
}

int main(int argc, char *argv[])
{
    cudaSetDevice(0);
    ros::init(argc,argv,"GetImg");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    tensorrt::init();

    camera::Camera cam;
    cv::Mat img;
    image_transport::Publisher pub = it.advertise("/img", 1);
    while(ros::ok()){
        cam.ReadImg(img);
        if(img.empty()) continue;
        cv::Mat resized_image;
        cv::Size new_size(640, 640); // the new size of the image
        cv::resize(img, resized_image, new_size, 0, 0, cv::INTER_LINEAR); // the resize function

        tensorrt::infer(resized_image);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tensorrt::res).toImageMsg();
        pub.publish(msg);

        ros::spinOnce();
    }

    return 0;
}
