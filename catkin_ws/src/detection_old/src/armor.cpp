#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <algorithm>
#include "armor.h"

bool Armor::cmp(Armor &x, Armor &y)
{
    return x.area() > y.area();
}

Armor::Armor(const cv::RotatedRect& left, const cv::RotatedRect& right, int _type)
{
    cv::RotatedRect new_left(left.center, cv::Size(left.size.width, left.size.height * 2), left.angle);
    cv::RotatedRect new_right(right.center, cv::Size(right.size.width, right.size.height * 2), right.angle);
    cv::Point2f points_l[4], points_r[4];
    new_left.points(points_l);
    new_right.points(points_r);
    upper_l = points_l[2], lower_l = points_l[3], upper_r = points_r[1], lower_r = points_r[0];
    center.x = (upper_l.x + lower_r.x) / 2;
    center.y = (upper_l.y + lower_r.y) / 2;
    type = _type;
}

Armor::Armor()
{
    upper_l = cv::Point2f(0, 0), lower_l = cv::Point2f(0, 0), upper_r = cv::Point2f(0, 0), lower_r = cv::Point2f(0, 0);
    center = cv::Point2f(0, 0);
}

bool Armor::empty()
{
    if (area() == 0)
        return true;
    return false;
}

std::vector<cv::Point2f> Armor::points()
{
    std::vector<cv::Point2f> vertices;
    vertices.push_back(lower_l);
    vertices.push_back(upper_l);
    vertices.push_back(upper_r);
    vertices.push_back(lower_r);
    return vertices;
}

std::vector<cv::Point2i> Armor::coordinates()
{
    std::vector<cv::Point2f> vertices = this->points();
    std::vector<cv::Point2i> coords;
    for (size_t i = 0; i < vertices.size(); i++)
        coords.push_back(cv::Point2i(std::max((int)vertices[i].x, 0), std::max((int)vertices[i].y, 0)));
    return coords;
}


//def calcArea(x1, y1, x2, y2, x3, y3):
//    return 2 * (x1 * y2 - x1 * y3 + x2 * y3 - x2 * y1 + x3 * y1 - x3 * y2)
double Armor::area()
{
    return abs(2 * (upper_l.x * upper_r.y - upper_l.x * lower_l.y + upper_r.x * lower_l.y 
        - upper_r.x * upper_l.y + lower_l.x * upper_l.y - lower_l.x * upper_r.y));
}