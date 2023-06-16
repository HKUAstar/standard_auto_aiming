#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "hikrobot_camera.hpp"
#include "armor.h"
#include "plain_detect.h"
#include "label_recognizer.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "recognition_test");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Publisher pub = it.advertise("binarized_image", 10);
    LabelRecognizer recognizer("model.pt");

    cv::Mat src;
    camera::Camera image_capture;
    int cnt = 0;
    while (ros::ok())
    {
        image_capture.ReadImg(src);
        if (src.empty())
            continue;
        std::vector<Armor> armors = Detector::analyze(src, 'b');
        std::vector<cv::Mat> result = recognizer.preprocess(src, armors);
        std::cout << "Got " << result.size() << " images." << std::endl;
        for (cv::Mat img: result)
        {
            auto img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
            pub.publish(img_msg);
        }
        ros::spinOnce();
    }
}