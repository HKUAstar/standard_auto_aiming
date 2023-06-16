#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <msgs/Armors.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <ctime>
#include "tools.h"

#define PITCH_OFFSET 0.0

cv::Mat camera_mtx, camera_dist;
cv::Point3f center(0, 0, 0);
Controller* controller;
PredictSpeed predictor;

int cnt = 0;

void callback(const msgs::Armors::ConstPtr& msg) {
    std::cout << "Get armors: " << msg->count << std::endl;
    if (msg->count == 0)
        return;
    msgs::ArmorItem armor = msg->items[0];
    ros::Time time0 = ros::Time::now();
    cv::Mat p_img = (cv::Mat_<float>(4, 2) << armor.x1, armor.y1, armor.x2, armor.y2, armor.x3, armor.y3, armor.x4, armor.y4);
    std::cout << "Got armor position at: " << p_img.row(0) << " " << p_img.row(1) << " " << p_img.row(2) << " " << p_img.row(3) << std::endl;
    cv::Mat rvec, tvec;
    pixel_to_cam(p_img, camera_mtx, camera_dist, rvec, tvec);
    std::cout << "Position in camera coordinate system: " << tvec.at<float>(0) << " " << tvec.at<float>(1) << " " << tvec.at<double>(2) << std::endl;
    cv::Mat predicted_tvec = predictor.predict(tvec).rowRange(0, 3);
    std::cout << "Predicted position: " << predicted_tvec.at<float>(0) << " " << predicted_tvec.at<float>(1) << " " << predicted_tvec.at<float>(2) << std::endl;
    if (predicted_tvec.at<float>(2) == 0) {
        return;
    }
    predicted_tvec -= cv::Mat(center);
    std::cout << predicted_tvec.at<float>(0) << " " << predicted_tvec.at<float>(2) << std::endl;
    float yaw_angle = std::atan(predicted_tvec.at<float>(0) / predicted_tvec.at<float>(2));
    float pitch_angle = std::atan(predicted_tvec.at<float>(1) / predicted_tvec.at<float>(2));
    std::cout << "Yaw: " << yaw_angle / CV_PI * 180 << " degrees ";
    std::cout << (yaw_angle > 0 ? "right" : "left") << ", Pitch: " << pitch_angle / CV_PI * 180 << " degrees ";
    std::cout << (pitch_angle > 0 ? "down" : "up") << std::endl;
    if (++cnt % 4 == 0)
        controller->moveGimbal(-yaw_angle, pitch_angle);
    ros::Time time1 = ros::Time::now();
    std::cout << (time1 - time0).toSec() << std::endl;
}

void adjustment_callback(const geometry_msgs::Vector3::ConstPtr& msg) {
    center += cv::Point3f(msg->x, msg->y, msg->z);
    std::cout << "Received adjustment: " << msg->x << " " << msg->y << " " << msg->z << std::endl;
    std::cout << "Current center: " << center << std::endl;
}

void listener() {
    ros::NodeHandle nh;
    ros::Subscriber sub_armors = nh.subscribe("armors", 1, callback);
    ros::Subscriber sub_adjustment = nh.subscribe("auto_aiming_adjustment", 1, adjustment_callback);
    ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_aiming_pnp");
    cv::FileStorage fs("/home/astar/catkin_ws/src/Sentry/detection/src/params/camera_old_8mm.yml", cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_mtx;
    if (camera_mtx.empty()) {
        std::cerr << "Failed to read camera matrix from file." << std::endl;
        return -1;
    }
    fs["distortion_coefficients"] >> camera_dist;
    if (camera_dist.empty()) {
        std::cerr << "Failed to read distortion coefficients from file." << std::endl;
        return -1;
    }
    controller = new Controller();
    predictor = PredictSpeed();
    listener();
    return 0;
}