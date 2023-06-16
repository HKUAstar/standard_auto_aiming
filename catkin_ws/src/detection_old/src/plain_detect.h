#ifndef PLAIN_DETECT_H
#define PLAIN_DETECT_H

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include "armor.h"

double point_distance(cv::Point2f l, cv::Point2f r);

class Detector
{
private:
    static const int brightness_threshold = 160;
    static const int light_min_area = 100.0;
    static constexpr double light_max_angle = 45.0;
    static constexpr double light_contour_min_solidity = 0.3; // 0.5
    static constexpr double light_max_ratio = 0.6; // 0.4
    static constexpr double light_color_detect_extend_ratio = 1.1;
    static constexpr double light_max_angle_diff = 5.0; // 6.0
    static constexpr double light_max_length_diff_ratio = 0.3; // 0.2
    static constexpr double light_max_y_diff_ratio = 2.0;
    static constexpr double light_min_x_diff_ratio = 0.5;
    static constexpr double armor_min_aspect_ratio = 1.0;
    static constexpr double armor_small_max_aspect_ratio = 2.5; // 5.0
    static constexpr double armor_big_max_aspect_ratio = 4.5;

    static cv::Mat separateColors(cv::Mat img, char color);
    static cv::Mat binarization(cv::Mat img);
    static std::vector<std::vector<cv::Point>> getContours(cv::Mat img);
    static cv::RotatedRect& adjustRec(cv::RotatedRect& rec);
    static bool compareLightIndicators(cv::RotatedRect l1, cv::RotatedRect l2);
    static std::vector<std::vector<cv::Point>> filterContours(std::vector<std::vector<cv::Point>>& light_contours, std::vector<cv::RotatedRect>& light_info);
    static std::vector<Armor> matchArmor(std::vector<cv::RotatedRect>& light_info);
    static void showArmor(cv::Mat img, std::vector<Armor> armors);

public:
    static std::vector<Armor> analyze(cv::Mat img, char color);
};

#endif
