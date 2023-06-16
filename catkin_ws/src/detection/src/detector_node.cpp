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
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "hikrobot_camera.hpp"

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"

//std
#include <memory>
#include <iostream>
#include <fstream>
#include <array>
#include <string>
#include <sstream>
#include <cmath>

//local
#include "armor.hpp"

#include "detector.hpp"

#include "number_classifier.hpp"

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
ros::Publisher pixel_pub;
ros::Publisher binary_pub;

std::vector<Armor> detectArmors(const cv::Mat& img)
{
    // Convert ROS img to cv::Mat
    // cv_bridge::CvImagePtr cv_ptr;
    // cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
    // auto img = cv_ptr->image;

    // Update params
    detector_->binary_thres = nh->param<int>("binary_thres", 160);

    detector_->detect_color = nh->param<int>("detect_color", 0);
    detector_->classifier->threshold = nh->param<double>("classifier_threshold", 0.7);

    auto armors = detector_->detect(img);

    // auto final_time = ros::Time::now();
    // auto latency = (final_time - img_msg->header.stamp).toSec() * 1000;
    // ROS_INFO("Latency: %fms", latency);
    return armors;
}

void annotate(cv::Mat &img, int x ,int y, std::string label)
{

    cv::Point textPos(x, y - 5);
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.5;
    int fontThickness = 1;
    cv::putText(img, label, textPos, fontFace, fontScale, cv::Scalar(255, 0, 0), fontThickness);
}

class Timer{
private:
    clock_t last_time;
public:
    Timer():last_time(clock()){}
    double get(){
        double res = (double)(clock()-last_time)/CLOCKS_PER_SEC;
        last_time = clock();
        ROS_INFO("--------------------------- Time from last get: %lf", res);
        return res;
    }
}T;

void imageCallback(cv::Mat& img)
{

    T.get();
    // publish trasform frame
    static tf2_ros::TransformBroadcaster broadcaster;
    tf2::Quaternion qtn,qtn1,qtn2;

//下y, 右x, 前z
//左y, 前x, z上 
    geometry_msgs::TransformStamped tfs2;
    tfs2.header.frame_id = "camera";
    tfs2.header.stamp = ros::Time::now();
    tfs2.child_frame_id = "optical";
    tfs2.transform.translation.x = 0.0;
    tfs2.transform.translation.y = 0.0;
    tfs2.transform.translation.z = 0.0;
    qtn.setRPY(-M_PI/2, 0, -M_PI/2);
    tfs2.transform.rotation.x = qtn.getX();
    tfs2.transform.rotation.y = qtn.getY();
    tfs2.transform.rotation.z = qtn.getZ();
    tfs2.transform.rotation.w = qtn.getW();
    broadcaster.sendTransform(tfs2);

    geometry_msgs::TransformStamped tfs;
    tfs.header.frame_id = "robot";
    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = "camera";
    tfs.transform.translation.x = 0.2;
    tfs.transform.translation.y = 0.0;
    tfs.transform.translation.z = 0.0;
    qtn.setRPY(0,0,0);
    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();
    broadcaster.sendTransform(tfs);

    auto armors = detectArmors(img);
    
    //4-8ms
    // cv_bridge::CvImagePtr cv_ptr;
    // cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);

    //3-6ms
//    this is to show the binary image
    // if(armors.size()>0)
    // {
        // cv::Mat binary_img;
        // detector_->preprocessImage(img, binary_img);
        //armors[0].number_img;
        

    //    Convert binary image to ROS message
        // sensor_msgs::ImagePtr binaryImageMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", binary_img).toImageMsg();
        //Publish binary image message
        // binary_pub.publish(binaryImageMsg);
    //     return;
    // }
    for (auto armor : armors)
    {
        if (armor.number == "negative") continue;
        cv::Point pt1 = armor.left_light.top;
        cv::Point pt2 = armor.left_light.bottom;
        cv::Point pt3 = armor.right_light.top;
        cv::Point pt4 = armor.right_light.bottom;
        std::vector<cv::Point2f> pts;
        pts.push_back(pt1);
        pts.push_back(pt2);
        pts.push_back(pt3);
        pts.push_back(pt4);
        // cv::RotatedRect rect = cv::minAreaRect(pts);
        // cv::Point2f vertices[4];
        // rect.points(vertices);
        cv::line(img, pts[0], pts[3], cv::Scalar(0, 0, 255), 2);
        cv::line(img, pts[1], pts[2], cv::Scalar(0, 0, 255), 2);
        for (int i = 0; i < 4; i++)
        {
            cv::circle(img, pts[i], 5, cv::Scalar(255, 255, 255), 1);
        }
        annotate(img, pt3.x, pt3.y, ARMOR_TYPE_STR[static_cast<int>(armor.type)]+armor.classfication_result);
        // ROS_INFO("number detection: %s, %s", ARMOR_TYPE_STR[static_cast<int>(armor.type)].c_str(), armor.classfication_result.c_str());
    }
    // 2-3ms
    // Convert the OpenCV image to ROS image message
    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();

    // Publish the output image message
    img_pub.publish(output_msg);


    // calclate the armor message to be published to tracker
    my_msgs::Armors armors_msg;

    if (pnp_solver != nullptr)
    {
        armors_msg.header.seq = 1;
        armors_msg.header.stamp = ros::Time::now();
        armors_msg.header.frame_id = "optical";
        armors_msg.armors.clear();

        my_msgs::Armor armor_msg;
        for (const auto & armor : armors)
        {
            ROS_INFO("number: %s", armor.number.c_str());
            if (armor.number == "negative") continue;
            cv::Mat rvec, tvec;
            bool success = pnp_solver->solvePnP(armor, rvec, tvec);
            geometry_msgs::Vector3 centr;
            centr.x = armor.center.x;
            centr.y = armor.center.y;
            pixel_pub.publish(centr);
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
                tf::Matrix3x3 tf_rotation_matrix(
                    rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
                    rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
                    rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                    rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
                    rotation_matrix.at<double>(2, 2)
                );
                double roll, yaw, pitch;
                tf_rotation_matrix.getRPY(roll, pitch, yaw);
                
                tf::Quaternion tf_q;
                tf_rotation_matrix.getRotation(tf_q);
                tf::quaternionTFToMsg(tf_q, armor_msg.pose.orientation);

                // Fill the distance to image center
                armor_msg.distance_to_image_center = pnp_solver->calculateDistanceToCenter(armor.center);
                // ROS_INFO("Camera_xyz: (%.2f %.2f %.2f)", armor_msg.pose.position.x, armor_msg.pose.position.y, armor_msg.pose.position.z);
                // ROS_INFO("roll: %.2f pitch: %.2f yaw: %.2f", roll, pitch, yaw);
                // ROS_INFO("Roll:%.2f Pitch:%.2f Yaw:%.2f", roll, pitch, yaw);
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
        ROS_INFO("armor published: --------------------%d", armors_msg.armors.size());
        armors_pub.publish(armors_msg);

    }
}

void init_camera()
{
    std::array<double, 9> camera_matrix;
    camera_matrix[0] = 2.330698835934038470e+03;
    camera_matrix[1] = 0.0;
    camera_matrix[2] = 7.270127711784648454e+02;
    camera_matrix[3] = 0.0;
    camera_matrix[4] = 2.334376390554373756e+03;
    camera_matrix[5] = 5.414281030767742777e+02;
    camera_matrix[6] = 0.0;
    camera_matrix[7] = 0.0;
    camera_matrix[8] = 1.0;

    
    std::vector<double> dist_coeffs(5);
    dist_coeffs[0] = -9.99483071e-03;
    dist_coeffs[1] = -5.04382890e-01;
    dist_coeffs[2] = -3.35322236e-03;
    dist_coeffs[3] = -1.76932027e-03;
    dist_coeffs[4] = 2.50898644e+00;
    pnp_solver = std::make_unique<PnPSolver>(camera_matrix, dist_coeffs);
}

std::unique_ptr<Detector> initDetector()
{
    // threshold image to get the light 
    int binary_thres = 160;
    // red = 0, blue = 1
    int detect_color = 0;
    
    Detector::LightParams l_params(0.1, 0.4, 20.0);
    // // width / height
    // double min_ratio;
    // double max_ratio;
    // // vertical angle
    // double max_angle;

    Detector::ArmorParams a_params(0.5, 0.7, 4.5, 0, 0, 20.0);
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
    std::string pkg_path = ros::package::getPath("detection");
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

    init_camera();

    img_pub = nh->advertise<sensor_msgs::Image>("/detection/annotated_img", 10);
    armors_pub = nh->advertise<my_msgs::Armors>("/detection/armors", 10);
    pixel_pub = nh->advertise<geometry_msgs::Vector3>("/pixel", 10);
    binary_pub = nh->advertise<sensor_msgs::Image>("/detection/binary_img", 10);

    detector_ = initDetector();

    image_transport::ImageTransport it(*nh);
    camera::Camera cam;
    cv::Mat img;
    // image_transport::Publisher pub = it.advertise("/detection/img", 1);
    while(ros::ok())
    {
        cam.ReadImg(img);
        if(img.empty()) continue;
        // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        // pub.publish(msg);
        imageCallback (img);
        ros::spinOnce();
    }    
    return 0;
}
#endif