#ifndef TOOLS_H
#define TOOLS_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

class Controller
{
public:
    Controller(double initial_yaw = 0, double initial_pitch = 0);
    void moveGimbal(double yaw_angle, double pitch_angle, bool absolute = false);
    void shoot(int mode, int number = 1);
    void endshoot();
    bool empty();

private:
    ros::Publisher pub;
    ros::ServiceClient client;
    int shoot_count;
    double yaw_accumulation, pitch_accumulation;
};

class SpeedPredictor {
public:
    SpeedPredictor() {
        kf = cv::KalmanFilter(6, 3, 0);
        kf.measurementMatrix = (cv::Mat_<float>(3, 6) << 1, 0, 0, 0, 0, 0,
                                                           0, 1, 0, 0, 0, 0,
                                                           0, 0, 1, 0, 0, 0);
        kf.transitionMatrix = (cv::Mat_<float>(6, 6) << 1, 0, 0, 1, 0, 0,
                                                         0, 1, 0, 0, 1, 0,
                                                         0, 0, 1, 0, 0, 1,
                                                         0, 0, 0, 1, 0, 0,
                                                         0, 0, 0, 0, 1, 0,
                                                         0, 0, 0, 0, 0, 1);
        kf.measurementNoiseCov = cv::Mat::eye(3, 3, CV_32F) * 1;
        kf.processNoiseCov = cv::Mat::eye(6, 6, CV_32F) * 0.01;
    }

    cv::Mat predict(cv::Mat tar) {
        cv::Mat measured = tar.reshape(1, 3);
        measured.convertTo(measured, CV_32F);
        cv::Mat predicted = kf.predict();
        kf.correct(measured);
        return predicted;
    }

    void record(cv::Mat tar) {
        cv::Mat measured = tar.reshape(3, 1);
        kf.correct(measured);
    }

    void reset(cv::Mat tar) {
        kf = cv::KalmanFilter(6, 3, 0);
        kf.measurementMatrix = (cv::Mat_<float>(3, 6) << 1, 0, 0, 0, 0, 0,
                                                           0, 1, 0, 0, 0, 0,
                                                           0, 0, 1, 0, 0, 0);
        kf.transitionMatrix = (cv::Mat_<float>(6, 6) << 1, 0, 0, 1, 0, 0,
                                                         0, 1, 0, 0, 1, 0,
                                                         0, 0, 1, 0, 0, 1,
                                                         0, 0, 0, 1, 0, 0,
                                                         0, 0, 0, 0, 1, 0,
                                                         0, 0, 0, 0, 0, 1);
        kf.measurementNoiseCov = cv::Mat::eye(3, 3, CV_32F) * 1;
        kf.processNoiseCov = cv::Mat::eye(6, 6, CV_32F) * 0.01;
        kf.statePre = (cv::Mat_<float>(6, 1) << tar.at<float>(0), tar.at<float>(1), tar.at<float>(2), 0, 0, 0);
        kf.statePost = (cv::Mat_<float>(6, 1) << tar.at<float>(0), tar.at<float>(1), tar.at<float>(2), 0, 0, 0);
    }

private:
    cv::KalmanFilter kf;
};

typedef SpeedPredictor PredictSpeed;

/*
cv::Mat world_to_cam(cv::Mat p_world, cv::Mat angles, cv::Mat translation) {
    cv::Mat mat = cv::Mat::zeros(3, 3, CV_32F);
    mat.at<float>(0, 0) = cos(angles.at<float>(0));
    mat.at<float>(0, 1) = -sin(angles.at<float>(0));
    mat.at<float>(1, 0) = sin(angles.at<float>(0));
    mat.at<float>(1, 1) = cos(angles.at<float>(0));
    mat.at<float>(2, 2) = 1;

    cv::Mat mat2 = cv::Mat::zeros(3, 3, CV_32F);
    mat2.at<float>(0, 0) = 1;
    mat2.at<float>(1, 1) = cos(angles.at<float>(1));
    mat2.at<float>(1, 2) = sin(angles.at<float>(1));
    mat2.at<float>(2, 1) = -sin(angles.at<float>(1));
    mat2.at<float>(2, 2) = cos(angles.at<float>(1));

    cv::Mat mat3 = cv::Mat::zeros(3, 3, CV_32F);
    mat3.at<float>(0, 0) = cos(angles.at<float>(2));
    mat3.at<float>(0, 2) = -sin(angles.at<float>(2));
    mat3.at<float>(1, 1) = 1;
    mat3.at<float>(2, 0) = sin(angles.at<float>(2));
    mat3.at<float>(2, 2) = cos(angles.at<float>(2));

    cv::Mat mat4 = mat3 * mat2 * mat;

    cv::Mat p_cam = mat4 * p_world + translation;
    return p_cam;
}

cv::Mat cam_to_world(cv::Mat p_cam, cv::Mat angles, cv::Mat translation) {
    cv::Mat mat = cv::Mat::zeros(3, 3, CV_32F);
    mat.at<float>(0, 0) = cos(angles.at<float>(0));
    mat.at<float>(0, 1) = -sin(angles.at<float>(0));
    mat.at<float>(1, 0) = sin(angles.at<float>(0));
    mat.at<float>(1, 1) = cos(angles.at<float>(0));
    mat.at<float>(2, 2) = 1;

    cv::Mat mat2 = cv::Mat::zeros(3, 3, CV_32F);
    mat2.at<float>(0, 0) = 1;
    mat2.at<float>(1, 1) = cos(angles.at<float>(1));
    mat2.at<float>(1, 2) = sin(angles.at<float>(1));
    mat2.at<float>(2, 1) = -sin(angles.at<float>(1));
    mat2.at<float>(2, 2) = cos(angles.at<float>(1));

    cv::Mat mat3 = cv::Mat::zeros(3, 3, CV_32F);
    mat3.at<float>(0, 0) = cos(angles.at<float>(2));
    mat3.at<float>(0, 2) = -sin(angles.at<float>(2));
    mat3.at<float>(1, 1) = 1;
    mat3.at<float>(2, 0) = sin(angles.at<float>(2));
    mat3.at<float>(2, 2) = cos(angles.at<float>(2));

    cv::Mat mat4 = mat * mat2 * mat3;

    cv::Mat p_world = mat4.inv() * (p_cam - translation);
    return p_world;
}
*/

void pixel_to_cam(cv::Mat &p_img, cv::Mat &camera_mtx, cv::Mat &camera_dist, cv::Mat &rvec, cv::Mat &tvec, int label = 3);

#endif
