#ifndef ARMOR_H
#define ARMOR_H

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

class Armor
{
public:
    Armor(const cv::RotatedRect& left, const cv::RotatedRect& right, int _type = 0);
    Armor();
    std::vector<cv::Point2f> points();
    std::vector<cv::Point2i> coordinates();
    double area();
    cv::Point2f center, upper_l, lower_l, upper_r, lower_r;
    bool empty();
    static bool cmp(Armor &x, Armor &y);
    int id = 0, label = 0, type = 0;
    bool operator!=(const Armor& other) const
    {
        return this->center != other.center || this->upper_l != other.upper_l || this->upper_r != other.upper_r 
            || this->lower_l != other.lower_l || this->lower_r != other.lower_r || this->id != other.id;
    }
    bool operator==(const Armor& other) const
    {
        return this->center == other.center && this->upper_l == other.upper_l && this->upper_r == other.upper_r 
            && this->lower_l == other.lower_l && this->lower_r == other.lower_r && this->id == other.id;
    }
};

#endif