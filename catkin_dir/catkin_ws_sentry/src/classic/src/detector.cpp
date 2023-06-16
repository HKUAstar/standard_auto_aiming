// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#include "ros/ros.h"

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <vector>

#include "detector.hpp"

Detector::Detector(const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a) : binary_thres(bin_thres), detect_color(color), l(l), a(a){}

double Detector::calculateBoundingBoxArea(std::vector<cv::Point2f>& points)
{
    // Define the 3D coordinates of the bounding box
    std::vector<cv::Point3f> objectPoints = {
        cv::Point3f(0, 0, 0),          // top-left
        cv::Point3f(0, image.rows, 0), // bottom-left
        cv::Point3f(image.cols, image.rows, 0), // bottom-right
        cv::Point3f(image.cols, 0, 0)  // top-right
    };

    // Estimate the pose of the object using PnP
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F); // identity matrix
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F); // no distortion
    cv::Mat rvec, tvec;
    cv::solvePnP(objectPoints, points, cameraMatrix, distCoeffs, rvec, tvec);

    // Calculate the dimensions of the bounding box
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    double width = cv::norm(R.col(0)) * image.cols;
    double height = cv::norm(R.col(1)) * image.rows;

    // Calculate the area of the bounding box
    double area = width * height;
    return area;
}

std::vector<Armor> Detector::detect(const cv::Mat & input)
{
    image = input;
    auto binary_img = preprocessImage(input);
    lights_ = findLights(input, binary_img);

    ROS_INFO("number light: %d", lights_.size());
    for (auto light : lights_)
    {
        ROS_INFO("pos : %f %f %f %f", light.length, light.width, light.top.x, light.top.y);
    }
    armors_ = matchLights(lights_);

    if (!armors_.empty()) {
        classifier->extractNumbers(input, armors_);
        classifier->classify(armors_);
    }

    return armors_;
}

cv::Mat Detector::preprocessImage(const cv::Mat & rgb_img)
{
    cv::Mat gray_img;
    cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);
    
    cv::Mat binary_img;
    cv::threshold(gray_img, binary_img, binary_thres, 255, cv::THRESH_BINARY);
    
    std::vector<std::pair<int,int> > vec;
    vec.emplace_back(std::make_pair(-1, -1));
    vec.emplace_back(std::make_pair(-1, 0));
    vec.emplace_back(std::make_pair(-1, 1));
    vec.emplace_back(std::make_pair(0, -1));
    vec.emplace_back(std::make_pair(0, 0));
    vec.emplace_back(std::make_pair(0, 1));
    vec.emplace_back(std::make_pair(1, -1));
    vec.emplace_back(std::make_pair(1, 0));
    vec.emplace_back(std::make_pair(1, 1));

    int rows = binary_img.rows, cols = binary_img.cols;
    cv::Mat pooled = cv::Mat::zeros(rows, cols, CV_8U);

    // Perform max pooling
    for (int i = 0; i < rows; i++)
        for (int j =0; j < cols; j++)
        {
            uchar max_val = 0;
            for (auto w : vec)
                if (0 <= i + w.first && i + w.first < rows && 0 <= j + w.second && j + w.second < cols)
                {
                    uchar pixel_val = binary_img.at<uchar>(i + w.first, j + w.second);
                    if (pixel_val > max_val) max_val = pixel_val;
                }
            pooled.at<uchar>(i, j) = max_val;
        }
    return pooled;
}

std::vector<Light> Detector::findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img)
{
    using std::vector;
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    vector<Light> lights;
    // this->debug_lights.data.clear();

    for (const auto & contour : contours)
    {
        if (contour.size() < 5) continue;

        auto r_rect = cv::minAreaRect(contour);
        auto light = Light(r_rect);

        if (isLight(light))
        {
            auto rect = light.boundingRect();
            // Avoid assertion failed
            if (0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rbg_img.cols && 0 <= rect.y &&
                0 <= rect.height && rect.y + rect.height <= rbg_img.rows)
            {
                int sum_r = 0, sum_b = 0;
                auto roi = rbg_img(rect);
                // Iterate through the ROI
                for (int i = 0; i < roi.rows; i++)
                {
                    for (int j = 0; j < roi.cols; j++)
                    {
                        if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
                            // if point is inside contour
                            sum_r += roi.at<cv::Vec3b>(i, j)[0];
                            sum_b += roi.at<cv::Vec3b>(i, j)[2];
                        }
                    }
                }
                // Sum of red pixels > sum of blue pixels ?
                light.color = sum_r > sum_b ? RED : BLUE;
                lights.emplace_back(light);
            }
        }
    }

    return lights;
}

bool Detector::isLight(const Light & light)
{
    // The ratio of light (short side / long side)
    float ratio = light.width / light.length;
    bool ratio_ok = l.min_ratio < ratio && ratio < l.max_ratio;

    bool angle_ok = light.tilt_angle < l.max_angle;

    bool is_light = ratio_ok && angle_ok;

    return is_light;
}

double Detector::calc_loss(const Light & light_1, const Light & light_2)
{
    double avg_light_length = (light_1.length + light_2.length) / 2.0;

    // difference of 2 lights' height
    double height_loss = std::fabs(light_1.length - light_2.length) / avg_light_length;

    // // Distance between the center of 2 lights (unit : light length)
    // float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
    // bool center_distance_ok = (a.min_small_center_distance <= center_distance && center_distance < a.max_small_center_distance) || (a.min_large_center_distance <= center_distance && center_distance < a.max_large_center_distance);

    // Angle of light
    cv::Point2f vec1 = light_1.top - light_1.bottom;
    cv::Point2f vec2 = light_2.top - light_2.bottom;
    double angle1 = std::fabs(std::atan(vec1.y / vec1.x)) / CV_PI * 180;
    double angle2 = std::fabs(std::atan(vec2.y / vec2.x)) / CV_PI * 180;
    if(vec1.x < 0) angle1 += 180;
    if(vec2.x < 0) angle2 += 180;
    double angle_diff_loss = std::fabs(angle1 - angle2);

    // // width and height check, ratio of height to width
    // double w_h_ratio = std::max(light_1.length, light_2.length) / cv::norm(diff);
    // const double min_w_r_ratio = 0.05 / 0.13 * 0.9;
    // bool width_height_ok = min_w_r_ratio < w_h_ratio;
    
    double alpha1 = 1000;
    double alpha2 = 1;
    double loss = alpha1 * height_loss + alpha2 * angle_diff_loss;
    return loss;
}

std::vector<Armor> Detector::matchLights(const std::vector<Light> & lights)
{
    std::vector<Armor> armors;
    std::vector<int> Match_id;
    std::vector<double> Match_loss;
    // Loop all the pairing of lights
    for (int id_1 = 0; id_1 < lights.size(); id_1++)
    {
        int best_id = -1;
        double best_loss = 100000;
        for (auto id_2 = 0; id_2 < id_1; id_2++)
        {
            if (lights[id_1].color != detect_color || lights[id_2].color != detect_color) continue;

            if (containLight(lights[id_1], lights[id_2], lights)) continue;
            
            auto type = isArmor(lights[id_1], lights[id_2]);
            if (type != ArmorType::INVALID)
            {
                double cur_loss = calc_loss(lights[id_1], lights[id_2]);
                if (cur_loss < best_loss)
                {
                    best_loss = cur_loss;
                    best_id = id_2;
                    if (cur_loss < Match_loss[id_2])
                    {
                        Match_id[id_2] = id_1;
                        Match_loss[id_2] = cur_loss;
                    }
                }
            }
        }
        Match_id.emplace_back(best_id);
        Match_loss.emplace_back(best_loss);
    }
    for(int id = 0; id < lights.size(); id++)
    {
        int u = id, v = Match_id[id];
        if (v == -1) continue;
        if (Match_id[v] == u && Match_id[u] == v)
        {
            auto type = isArmor(lights[u], lights[v]);

            auto armor = Armor(lights[u], lights[v]);
            armor.type = type;
            armors.emplace_back(armor);
        }
    }

    return armors;
}

// Check if there is another light in the boundingRect formed by the 2 lights
bool Detector::containLight(const Light & light_1, const Light & light_2, const std::vector<Light> & lights)
{
    auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
    auto bounding_rect = cv::boundingRect(points);

    for (const auto & test_light : lights)
    {
        if (test_light.center == light_1.center || test_light.center == light_2.center) continue;

        if (bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) || bounding_rect.contains(test_light.center))
        {
            return true;
        }
    }

    return false;
}

ArmorType Detector::isArmor(const Light & light_1, const Light & light_2)
{

    // Ratio of the length of 2 lights (short side / long side)
    //
    float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length : light_2.length / light_1.length;
    bool light_ratio_ok = light_length_ratio > a.min_light_ratio;

    // Distance between the center of 2 lights (unit : light length)
    float avg_light_length = (light_1.length + light_2.length) / 2;
    //1.15/0.75
    float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
    bool center_distance_ok = (a.min_small_center_distance <= center_distance && center_distance < a.max_small_center_distance) || (a.min_large_center_distance <= center_distance && center_distance < a.max_large_center_distance);

    // Angle of light center connection
    cv::Point2f diff = light_1.center - light_2.center;
    float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
    bool angle_ok = angle < a.max_angle;
    // if ((light_1.top.x - light_1.bottom.x) * (light_2.top.x - light_2.bottom.x) < 0)
    //     angle_ok = 0;
    
    // width and height check, ratio of height to width
    double w_h_ratio = std::max(light_1.length, light_2.length) / cv::norm(diff);
    const double min_w_r_ratio = 0.05 / 0.13 * 0.9;
    bool width_height_ok = min_w_r_ratio < w_h_ratio;

    bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;
    // 
    // Judge armor type
    ArmorType type;
    if (is_armor)
    {
        type = center_distance > a.min_large_center_distance ? ArmorType::LARGE : ArmorType::SMALL;
    }
    else
    {
        type = ArmorType::INVALID;
    }

    // // Fill in debug information
    // auto_aim_interfaces::msg::DebugArmor armor_data;
    // armor_data.type = ARMOR_TYPE_STR[static_cast<int>(type)];
    // armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
    // armor_data.light_ratio = light_length_ratio;
    // armor_data.center_distance = center_distance;
    // armor_data.angle = angle;
    // this->debug_armors.data.emplace_back(armor_data);

    return type;
}

// cv::Mat Detector::getAllNumbersImage()
// {
//     if (armors_.empty()) {
//         return cv::Mat(cv::Size(20, 28), CV_8UC1);
//     }
//     else
//     {
//         std::vector<cv::Mat> number_imgs;
//         number_imgs.reserve(armors_.size());
//         for (auto & armor : armors_) {
//         number_imgs.emplace_back(armor.number_img);
//     }
//     cv::Mat all_num_img;
//     cv::vconcat(number_imgs, all_num_img);
//     return all_num_img;
//     }
// }

// void Detector::drawResults(cv::Mat & img)
// {
//   // Draw Lights
//   for (const auto & light : lights_) {
//     cv::circle(img, light.top, 3, cv::Scalar(255, 255, 255), 1);
//     cv::circle(img, light.bottom, 3, cv::Scalar(255, 255, 255), 1);
//     auto line_color = light.color == RED ? cv::Scalar(255, 255, 0) : cv::Scalar(255, 0, 255);
//     cv::line(img, light.top, light.bottom, line_color, 1);
//   }

//   // Draw armors
//   for (const auto & armor : armors_) {
//     cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
//     cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2);
//   }

//   // Show numbers and confidence
//   for (const auto & armor : armors_) {
//     cv::putText(
//       img, armor.classfication_result, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8,
//       cv::Scalar(0, 255, 255), 2);
//   }
// }

