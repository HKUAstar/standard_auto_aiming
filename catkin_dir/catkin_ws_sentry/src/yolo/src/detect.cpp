#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "hikrobot_camera.hpp"

#include <chrono>
#include <fstream>
#include <iostream>
#include <getopt.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <ctime>
#include <string>

#define debug(x) (std::cout << (x) << std::endl)

const std::string PWD = "/home/sentry/catkin_ws/src/yolo/src/";

// using namespace Ort;

namespace yolo{
    Ort::Env *env;
    Ort::SessionOptions session_options;
    Ort::Session *session;
    Ort::Allocator* allocator;
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);    

    std::vector<const char*> input_names = {"images"};
    std::vector<const char*> output_names = {"output0"};
    std::vector<int64_t> input_shape = {1, 3, 640, 640};

    float cls_threshold = 0.25;
    float nms_threshold = 0.5;

    void init(){
        env = new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "test");

        OrtSessionOptionsAppendExecutionProvider_Tensorrt(session_options, 0);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

        session = new Ort::Session(*env, "/home/sentry/catkin_ws/src/yolo/src/models/best2.onnx", session_options);
        allocator = new Ort::Allocator(*session, memory_info);
    }

    void annotate(cv::Mat &img, cv::Rect rect, std::string label){
        cv::Scalar color(0, 255, 0);
        cv::rectangle(img, rect, color, 1.5);

        cv::Point textPos(rect.x, rect.y - 5);
        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.5;
        int fontThickness = 1;
        cv::putText(img, label, textPos, fontFace, fontScale, cv::Scalar(0, 0, 255), fontThickness);
    }

    void post_process(std::vector<Ort::Value> &output_values, cv::Mat &img){
        float* output_data = output_values[0].GetTensorMutableData<float>();
        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        std::vector<int> classes;
        for (int j = 0; j < 8400; j++){
            float mx = 0;
            int cls = 0;
            for(int i = 4; i < 21; i++){
                if(output_data[i*8400+j] > mx) mx = output_data[i*8400+j], cls = i-4;
                // std::cout << (output_data[i*8400+j]) << " ";
            }
            if(mx > cls_threshold){
                float x = output_data[j], y = output_data[1*8400+j], w = output_data[2*8400+j], h = output_data[3*8400+j];
                float left = x-w/2, top = y-h/2;
                cv::Rect rect(left, top, w, h);
                boxes.emplace_back(rect);
                scores.emplace_back(mx);
                classes.emplace_back(cls);
            }
        }
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, scores, 0.0, nms_threshold, indices);
        // debug(indices.size());
        for(auto id:indices){
            std::string label = std::to_string(classes[id])+"  "+std::to_string(scores[id]);
            // debug(label);
            // annotate(img, boxes[id], label);
            // std::cout << boxes[id].x << " " << boxes[id].y << " " << boxes[id].width << " " << boxes[id].height << std::endl;
        }
    }
        
    void infer(cv::Mat &origin){
        cv::Mat img;
        cv::resize(origin, origin, cv::Size(640, 640));
        // cv::cvtColor(origin, img, cv::COLOR_BGR2RGB);
        origin.convertTo(img, CV_32F, 1.0/255.0);
        cv::Mat blob = cv::dnn::blobFromImage(img, 1., cv::Size(640, 640), cv::Scalar(0, 0, 0), true);
        size_t input_data_length = blob.total() * blob.elemSize();
        int num_input_nodes = 1;
        std::vector<Ort::Value> input_values;
        input_values.emplace_back(Ort::Value::CreateTensor<float>(memory_info, blob.ptr<float>(), input_data_length, input_shape.data(), input_shape.size()));
        std::vector<Ort::Value> output_values = session->Run(Ort::RunOptions{nullptr}, input_names.data(), 
                                        input_values.data(), input_values.size(), output_names.data(),
                                        output_names.size()
        );
        post_process(output_values, origin);
    }

    void destroy(){
        delete env;
        delete session;
    }

    void warm_up(){
        std::cout << "Warming up..." << std::endl;
        cv::Mat img = cv::imread(PWD+"47.jpg", cv::IMREAD_UNCHANGED);
        // Warm up the inference process
        for(int i = 0; i < 20; i++) yolo::infer(img);
        std::cout << "Done." << std::endl;
    }

    void benchmark(){
        cv::Mat img = cv::imread(PWD+"47.jpg", cv::IMREAD_UNCHANGED);
        cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
        auto start = clock();
        for(int i = 0; i < 100; i++) yolo::infer(img);
        auto end = clock();
        debug((double)(end-start)/CLOCKS_PER_SEC);
    }
}




int main(int argc, char *argv[])
{
    ros::init(argc,argv,"GetImg");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    yolo::init();
    yolo::warm_up();
    // yolo::benchmark();

    camera::Camera cam;
    cv::Mat img;
    image_transport::Publisher pub = it.advertise("/img", 1);
    while(ros::ok()){
        auto start = clock();
        cam.ReadImg(img);
        if(img.empty()) continue;

        yolo::infer(img);
        auto end = clock();
        debug((double)(end-start)/CLOCKS_PER_SEC*1000);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        pub.publish(msg);

        ros::spinOnce();
    }

    return 0;
}
