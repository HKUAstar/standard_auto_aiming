#ifndef CLASSIC__DETECTOR_NODE_CPP_
#define CLASSIC__DETECTOR_NODE_CPP_

//opencv
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core/mat.hpp>

//ros1
#include "ros/ros.h"
#include "ros/package.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

#include <tf/LinearMath/Matrix3x3.h>
// #include <tf/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


//std
#include <memory>

//local
#include "armor.hpp"

#include "detector.hpp"
#include "detector.cpp"

#include "number_classifier.cpp"
#include "number_classifier.hpp"

#include "pnp_solver.cpp"
#include "pnp_solver.hpp"

#include "my_msgs/Armor.h"
#include "my_msgs/Armors.h"


ros::NodeHandle* nh;
cv::Point2f cam_center_;
std::shared_ptr<sensor_msgs::CameraInfo> cam_info_;
std::unique_ptr<Detector> detector_;
std::unique_ptr<PnPSolver> pnp_solver = nullptr;

ros::Subscriber img_sub;
ros::Subscriber cam_info_sub;

ros::Publisher img_pub;
ros::Publisher armors_pub;
std::vector<Armor> detectArmors(const sensor_msgs::ImageConstPtr& img_msg)
{
    // Convert ROS img to cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
    auto img = cv_ptr->image;

    // Update params
    detector_->binary_thres = nh->param<int>("binary_thres", 160);
    detector_->detect_color = nh->param<int>("detect_color", 0);
    detector_->classifier->threshold = nh->param<double>("classifier_threshold", 0.7);

    auto armors = detector_->detect(img);

    auto final_time = ros::Time::now();
    auto latency = (final_time - img_msg->header.stamp).toSec() * 1000;
    ROS_INFO("Latency: %fms", latency);
    return armors;
}

void imageCallback(const sensor_msgs::ImageConstPtr& img_msg)
{
    auto armors = detectArmors(img_msg);
    
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);

//   //  this is to show the binary image
//     auto img = cv_ptr->image;
//     auto binary_img = detector_->preprocessImage(img);
// //    Convert binary image to ROS message
//     sensor_msgs::ImagePtr binaryImageMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", binary_img).toImageMsg();
// //   Publish binary image message
//     img_pub.publish(binaryImageMsg);
//     return;


    for (auto armor : armors)
    {
        // if (armor.number == "negative") continue;
        cv::Point pt1 = armor.left_light.top;
        cv::Point pt2 = armor.left_light.bottom;
        cv::Point pt3 = armor.right_light.top;
        cv::Point pt4 = armor.right_light.bottom;
        std::vector<cv::Point2f> pts;
        pts.push_back(pt1);
        pts.push_back(pt2);
        pts.push_back(pt3);
        pts.push_back(pt4);
        cv::RotatedRect rect = cv::minAreaRect(pts);
        cv::Point2f vertices[4];
        rect.points(vertices);
        for (int i = 0; i < 4; i++)
        {
            cv::line(cv_ptr->image, vertices[i], vertices[(i+1)%4], cv::Scalar(0, 0, 255), 2);
        }
        ROS_INFO("number detection: %s", armor.classfication_result.c_str());
    }
    // Convert the OpenCV image to ROS image message
    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();

    // Publish the output image message
    img_pub.publish(output_msg);


    // calclate the armor message to be published to tracker

    my_msgs::Armors armors_msg;

    if (pnp_solver != nullptr)
    {
        armors_msg.header = img_msg->header;
        armors_msg.armors.clear();

        my_msgs::Armor armor_msg;
        for (const auto & armor : armors)
        {
            cv::Mat rvec, tvec;
            bool success = pnp_solver->solvePnP(armor, rvec, tvec);
            if (success)
            {
                // Fill basic info
                armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
                armor_msg.number = armor.number;

                // Fill pose
                armor_msg.pose.position.x = tvec.at<double>(0);
                armor_msg.pose.position.y = tvec.at<double>(1);
                armor_msg.pose.position.z = tvec.at<double>(2);
                // rvec to 3x3 rotation matrix
                cv::Mat rotation_matrix;
                cv::Rodrigues(rvec, rotation_matrix);
                // rotation matrix to quaternion
                tf2::Matrix3x3 tf2_rotation_matrix(
                rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
                rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
                rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
                rotation_matrix.at<double>(2, 2));
                tf2::Quaternion tf2_q;
                tf2_rotation_matrix.getRotation(tf2_q);
                armor_msg.pose.orientation = tf2::toMsg(tf2_q);

                // Fill the distance to image center
                armor_msg.distance_to_image_center = pnp_solver->calculateDistanceToCenter(armor.center);

                // // Fill the markers
                // armor_marker_.id++;
                // armor_marker_.scale.y = armor.type == ArmorType::SMALL ? 0.135 : 0.23;
                // armor_marker_.pose = armor_msg.pose;
                // text_marker_.id++;
                // text_marker_.pose.position = armor_msg.pose.position;
                // text_marker_.pose.position.y -= 0.1;
                // text_marker_.text = armor.classfication_result;
                
                armors_msg.armors.emplace_back(armor_msg);
                // marker_asrray_.markers.emplace_back(armor_marker_);
                // marker_array_.markers.emplace_back(text_marker_);
            } 
            else
            {
                ROS_INFO("PnP failed!");
            }
        }

        // Publishing detected armors
        armors_pub.publish(armors_msg);

    }
}

// void init_camera()
// {
//     cam_center_ = cv::Point2f(camera_info.k[2], camera_info.k[5]);
//     cam_info_ = std::make_shared<sensor_msgs::CameraInfo>(camera_info);
//     pnp_solver = std::make_unique<PnPSolver>(camera_info.k, camera_info.d);
//     cam_info_sub.shutdown();
// }

std::unique_ptr<Detector> initDetector()
{
    // threshold image to get the light 
    int binary_thres = 160;
    // blue = 0, red = 1
    int detect_color = 1;
    
    Detector::LightParams l_params(0.1, 0.4, 40.0);
    // // width / height
    // double min_ratio;
    // double max_ratio;
    // // vertical angle
    // double max_angle;

    Detector::ArmorParams a_params(0.7, 1, 2.5, 0, 0, 30.0);
    // double min_light_ratio;
    // // light pairs distance
    // double min_small_center_distance;
    // double max_small_center_distance;
    // double min_large_center_distance;
    // double max_large_center_distance;
    // // horizontal angle

    // Init detector 
    std::unique_ptr<Detector> detector = std::make_unique<Detector>(binary_thres, detect_color, l_params, a_params);

    // Init classifier
    std::string pkg_path = ros::package::getPath("classic");
    std::string model_path = pkg_path + "/model/mlp.onnx";
    std::string label_path = pkg_path + "/model/label.txt";
    //classify_threshold
    double threshold = 0.7;
    std::vector<std::string> ignore_classes;
    // ros::param::get("ignore_classes", ignore_classes); 
    detector->classifier = std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);
    return detector;
}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "detector_node");

    nh = new ros::NodeHandle();
    
    ROS_INFO("Starting DetectorNode!");

    // init_camera();

    img_sub = nh->subscribe("/classic/img", 10, imageCallback);

    img_pub = nh->advertise<sensor_msgs::Image>("/classic/annotated_img", 10);
    armors_pub = nh->advertise<my_msgs::Armors>("/armors", 10);

    detector_ = initDetector();
    
    ros::spin();
    
    return 0;
}

#endif