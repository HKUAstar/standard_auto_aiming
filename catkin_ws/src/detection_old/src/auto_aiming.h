#ifndef AUTO_AIMING_H
#define AUTO_AIMING_H

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "armor.h"
#include "tools.h"
#include "label_recognizer.h"

// ### Hyperparameters ###
const int IMG_W = 1440;
const int IMG_H = 1080;
const double MAX_PITCH_IDLE_RANGE = 0.4;
const double PITCH_OFFSET = 0.000; // for raising the head
const double MAX_YAW = 0.100;
const double MAX_PITCH = 0.025;
const double MIN_YAW = 0.005;
const double MIN_PITCH = 0.005;
const double YAW_RANGE_1 = 700; // velocity mode
const double YAW_RANGE_2 = 250; // acceleration mode
const double PITCH_RANGE = 200;
const double MAX_YAW_ACC = 0.110;
const double MIN_YAW_ACC = 0.040;
const double MAX_PITCH_ACC = 0.035;
const double MIN_PITCH_ACC = 0.010;
const double ACC_MULTIPLIER = 0.025;
const int TARGET_ZONE = 8; // radius in pixels
const int MIN_TARGET_AREA = 3000;
//const int TARGET_AREA_EPS = 100;
//const int MODE_EPS = 3;
const double IMAGE_GAMMA = 3;
const double MAX_TARGET_FRAME_DISTANCE = 100.00;
const double ROTATING_SPEED = 80;
const bool READ_IMAGE_FROM_TOPIC = false;
// ### Hyperparameters ###

const int SHOOT_X = IMG_W / 2;
const int SHOOT_Y = IMG_H / 2;

class AutoAiming
{
public:
    AutoAiming();
    void aim(cv::Mat &img, char color);
    void publishArmorImage(cv::Mat &img, Armor armor);
    void publishStatus();
    void adjustCenter(cv::Point3f adjustment);
    Controller* controller;
    ros::Publisher armor_pub;
    image_transport::Publisher image_pub;
    void publishArmor();
    ros::ServiceClient recognition_client;
    int tar_id;
    // void publishAnnotatedImage(Mat img, vector<Armor> armors, vector<int> tar, 
    //     vector<int> pred_tar, Point2d act);

private:
    void shootTarget(Armor tar);
    void traceTarget(Armor tar);
    Armor pickTarget(std::vector<Armor> &armors);
    void matchArmors(cv::Mat &img, std::vector<Armor> &armors);
    void publishAnnotatedImage(cv::Mat img, std::vector<Armor> armors);
    unsigned int identifyArmor(cv::Mat &img, Armor new_armor);
    bool needPublish();
    std::vector<Armor> last_armors;
    SpeedPredictor predictor;
    LabelRecognizer recognizer = LabelRecognizer("model.pt");
    Armor last_tar;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time;
    cv::Point3f center;
    cv::Point2i pred_tar;
    cv::Point2d last_act, act;
    std::vector<Armor> new_armors;
    cv::Mat camera_mtx, camera_dist;
    //bool x_on_target;
    //int mode;
    int manual_label;
    int missed_tar;
    double idle_angle, cooldown;
};

cv::Mat changeGamma(cv::Mat &img, double gamma);

#endif
