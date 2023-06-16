#ifndef ARMOR_DETECTOR__DETECTOR_HPP_
#define ARMOR_DETECTOR__DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>

#include "armor.hpp"
#include "number_classifier.hpp"

class Detector
{
public:
    struct LightParams
    {
        // width / height
        double min_ratio;
        double max_ratio;
        // vertical angle
        double max_angle;
        LightParams (double _min_ratio, double _max_ratio, double _max_angle)
        {
            min_ratio = _min_ratio;
            max_ratio = _max_ratio;
            max_angle = _max_angle;
        }
    };

    struct ArmorParams
    {
        double min_light_ratio;
        // light pairs distance
        double min_small_center_distance;
        double max_small_center_distance;
        double min_large_center_distance;
        double max_large_center_distance;
        // horizontal angle
        double max_angle;
        ArmorParams (double _min_light_ratio, double _min_small_center_distance, double _max_small_center_distance, double _min_large_center_distance, double _max_large_center_distance, double _max_angle)
        {
            min_light_ratio = _min_light_ratio;
            min_small_center_distance = _min_small_center_distance;
            max_small_center_distance = _max_small_center_distance;
            min_large_center_distance = _min_large_center_distance;
            max_large_center_distance = _max_large_center_distance;
            max_angle = _max_angle;
        }
    };
    
    Detector(const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a);
    double calculateBoundingBoxArea(std::vector<cv::Point2f>& points);
    std::vector<Armor> detect(const cv::Mat & input);

    void preprocessImage(const cv::Mat &, cv::Mat &);
    std::vector<Light> findLights(const cv::Mat & rgb_img, const cv::Mat & binary_img);
    std::vector<Armor> matchLights(const std::vector<Light> & lights);
    double calc_loss(const Light & light_1, const Light & light_2);
    // // For debug usage
    // cv::Mat getAllNumbersImage();
    // void drawResults(cv::Mat & img);

    int binary_thres;
    int detect_color;
    LightParams l;
    ArmorParams a;
    cv::Mat image;
    std::unique_ptr<NumberClassifier> classifier;


private:
    bool isLight(const Light & possible_light);
    bool containLight(const Light & light_1, const Light & light_2, const std::vector<Light> & lights);
    ArmorType isArmor(const Light & light_1, const Light & light_2);

    
    std::vector<Light> lights_;
    std::vector<Armor> armors_;
};


#endif
