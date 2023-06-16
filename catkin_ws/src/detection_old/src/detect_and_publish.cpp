#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Vector3.h>
#include <msgs/Armors.h>
#include <msgs/ArmorItem.h>
#include "armor.h"
#include "plain_detect.h"
#include "hikrobot_camera.hpp"
using namespace cv;
using namespace std;

bool test_speed = true, test_acc = false;

int cnt = 0;

double point_distance(Point2f l, Point2f r)
{
    return sqrt(pow(l.x - r.x, 2) + pow(l.y - r.y, 2));
}

bool Armor::cmp(Armor &x, Armor &y)
{
    return x.area() > y.area();
}

Armor::Armor(const RotatedRect& left, const RotatedRect& right, int _type)
{
    RotatedRect new_left(left.center, Size(left.size.width, left.size.height * 2), left.angle);
    RotatedRect new_right(right.center, Size(right.size.width, right.size.height * 2), right.angle);
    Point2f points_l[4], points_r[4];
    new_left.points(points_l);
    new_right.points(points_r);
    upper_l = points_l[2], lower_l = points_l[3], upper_r = points_r[1], lower_r = points_r[0];
    center.x = (upper_l.x + lower_r.x) / 2;
    center.y = (upper_l.y + lower_r.y) / 2;
    type = _type;
}

Armor::Armor()
{
    upper_l = Point2f(0, 0), lower_l = Point2f(0, 0), upper_r = Point2f(0, 0), lower_r = Point2f(0, 0);
    center = Point2f(0, 0);
}

bool Armor::empty()
{
    if (area() == 0)
        return true;
    return false;
}

vector<Point2f> Armor::points()
{
    vector<Point2f> vertices;
    vertices.push_back(lower_l);
    vertices.push_back(upper_l);
    vertices.push_back(upper_r);
    vertices.push_back(lower_r);
    return vertices;
}

vector<Point2i> Armor::coordinates()
{
    vector<Point2f> vertices = this->points();
    vector<Point2i> coords;
    for (size_t i = 0; i < vertices.size(); i++)
        coords.push_back(Point2i(max((int)vertices[i].x, 0), max((int)vertices[i].y, 0)));
    return coords;
}


//def calcArea(x1, y1, x2, y2, x3, y3):
//    return 2 * (x1 * y2 - x1 * y3 + x2 * y3 - x2 * y1 + x3 * y1 - x3 * y2)
double Armor::area()
{
    return abs(2 * (upper_l.x * upper_r.y - upper_l.x * lower_l.y + upper_r.x * lower_l.y 
        - upper_r.x * upper_l.y + lower_l.x * upper_l.y - lower_l.x * upper_r.y));
}

Mat Detector::separateColors(Mat img, char color)
{
    vector<Mat> channels;
    split(img, channels);

    Mat gray_img;

    if (color == 'r')
        gray_img = channels.at(2) - channels.at(0);
    else
        gray_img = channels.at(0) - channels.at(2);

    return gray_img;
}

Mat Detector::binarization(Mat img)
{
    Mat bin_bright_img;

    threshold(img, bin_bright_img, brightness_threshold, 255, THRESH_BINARY);

    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));

    dilate(bin_bright_img, bin_bright_img, element);
    //imshow("Binarization", bin_bright_img);
    //waitKey();

    return bin_bright_img;
}

vector<vector<Point>> Detector::getContours(Mat img)
{
    vector<vector<Point>> light_contours;
    findContours(img, light_contours, RetrievalModes::RETR_EXTERNAL, ContourApproximationModes::CHAIN_APPROX_SIMPLE);

    return light_contours;
}

RotatedRect& Detector::adjustRec(RotatedRect& rec)
{
    using std::swap;

    float& width = rec.size.width;
    float& height = rec.size.height;
    float& angle = rec.angle;

    while (angle >= 90.0) angle -= 180.0;
    while (angle < -90.0) angle += 180.0;

    if (angle >= 45.0)
    {
        swap(width, height);
        angle -= 90.0;
    }
    else if (angle < -45.0)
    {
            swap(width, height);
            angle += 90.0;
    }

    return rec;
}

bool Detector::compareLightIndicators(RotatedRect l1, RotatedRect l2)
{
    return l1.center.x < l2.center.x;
}

vector<vector<Point>> Detector::filterContours(vector<vector<Point>>& light_contours, vector<RotatedRect>& light_info)
{
    vector<vector<Point>> remaining_contours;
    for (const auto& contour:light_contours)
    {
        float light_contour_area = contourArea(contour);
        if (light_contour_area < light_min_area)
            continue;
        if (contour.size() < 5)
            continue;
        RotatedRect light_rec = fitEllipse(contour);
        adjustRec(light_rec);
        if (light_rec.size.width / light_rec.size.height > light_max_ratio ||
            light_contour_area / light_rec.size.area() < light_contour_min_solidity)
                continue;
        light_rec.size.width *= light_color_detect_extend_ratio;
        light_rec.size.height *= light_color_detect_extend_ratio;
        
        light_info.push_back(RotatedRect(light_rec));
        remaining_contours.push_back(contour);
    }
    return remaining_contours;
}

vector<Armor> Detector::matchArmor(vector<RotatedRect>& light_info)
{
    vector<Armor> armors;

    sort(light_info.begin(), light_info.end(), compareLightIndicators);
    for (size_t i = 0; i < light_info.size(); i++)
    {
        const RotatedRect& left = light_info[i];
        for (size_t j = i + 1; j < light_info.size(); j++)
        {
            const RotatedRect& right = light_info[j];
            
            double angle_diff = abs(left.angle - right.angle);
            double len_diff_ratio = abs(left.size.height - right.size.height) / max(left.size.height, right.size.height);
            
            if (angle_diff > light_max_angle_diff || len_diff_ratio > light_max_length_diff_ratio)
                continue;

            double dis = point_distance(left.center, right.center);
            double mean_len = (left.size.height + right.size.height) / 2;
            double x_diff_ratio = abs(left.center.x - right.center.x) / mean_len;
            double y_diff_ratio = abs(left.center.y - right.center.y) / mean_len;
            double dis_ratio = dis / mean_len;
            if (y_diff_ratio > light_max_y_diff_ratio || 
                x_diff_ratio < light_min_x_diff_ratio ||
                dis_ratio > armor_big_max_aspect_ratio ||
                dis_ratio < armor_min_aspect_ratio)
                continue;
            Armor armor(left, right);
            armors.push_back(armor);
        }
    }
    return armors;
}

void Detector::showArmor(Mat img, vector<Armor> armors)
{
    for (size_t i = 0; i < armors.size(); i++)
    {
        circle(img, armors[i].center, 1, Scalar(0, 255, 0), 10);
        vector<Point2i> armor_points = armors[i].coordinates();
        polylines(img, armor_points, true, Scalar(0, 255, 0), 1);
    }
    imshow("Armor", img);
    waitKey();
}

/*
void Detector::drawAllContours(Mat img, vector<vector<Point>> light_contours)
{
    for (size_t i = 0; i < light_contours.size(); i++)
        drawContours(img.clone(), light_contours, i, Scalar(0, 0, 255), 1);
}
*/

vector<Armor> Detector::analyze(Mat img, char color)
{
    //Mat debug_img = img.clone();
    img = separateColors(img, color);
    img = binarization(img);
    //Mat bin_img = img.clone();
    vector<vector<Point>> contours = getContours(img);

    vector<RotatedRect> light_info;
    contours = filterContours(contours, light_info);

    //drawAllContours(debug_img, contours);

    vector<Armor> armors = matchArmor(light_info);

    //cout << "Found " << armors.size() << " armor(s)" << endl;
    //if (test_acc) showArmor(debug_img, armors);
    
    return armors;
}

ros::Publisher pub;
image_transport::Publisher image_pub;

void publish_annotated_image(Mat img, msgs::Armors armors)
{
    int aim_point_x = 597, aim_point_y = 293;
    line(img, Point(aim_point_x - 20, aim_point_y), Point(aim_point_x + 20, aim_point_y), Scalar(0, 255, 0), 3);
    line(img, Point(aim_point_x, aim_point_y - 20), Point(aim_point_x, aim_point_y + 20), Scalar(0, 255, 0), 3); 
    for (msgs::ArmorItem armor: armors.items)
    {
        vector<Point2i> armor_points = {Point2i(armor.x1, armor.y1), Point2i(armor.x2, armor.y2), Point2i(armor.x3, armor.y3), Point2i(armor.x4, armor.y4)};
        polylines(img, armor_points, true, Scalar(0, 255, 0), 3);
    }
    auto img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    image_pub.publish(img_msg);
}

void callback(const sensor_msgs::ImageConstPtr& msg)
{
    auto start = chrono::high_resolution_clock::now();
    char color = 'b';
    msgs::Armors result_msg;
    //result_msg.header.stamp = ros::Time::now();
    
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    
    Mat img = cv_ptr->image;
    
    //resize(img, img, Size(img_width, img_height));
    vector<Armor> result = Detector::analyze(img, color);

    msgs::ArmorItem item;
    //geometry_msgs::Vector3 result_msg;
    
    result_msg.count = result.size();
    result_msg.items.clear();
    
    for (size_t i = 0; i < result.size(); i++)
    {
        item.x1 = max((int)result[i].upper_l.x, 0);
        item.y1 = max((int)result[i].upper_l.y, 0);
        item.x2 = max((int)result[i].lower_l.x, 0);
        item.y2 = max((int)result[i].lower_l.y, 0);
        item.x3 = max((int)result[i].lower_r.x, 0);
        item.y3 = max((int)result[i].lower_r.y, 0);
        item.x4 = max((int)result[i].upper_r.x, 0);
        item.y4 = max((int)result[i].upper_r.y, 0);
        item.area = (int)result[i].area();
        item.id = 0; // to be changed with identification
        result_msg.items.push_back(item);
    }
    auto end = chrono::high_resolution_clock::now();
    cout << "Published result of " << result_msg.count << " armors" << endl;
    //if (result_msg.count > 0)
    //{
    //    cout << item.cx << " " << item.cy << endl;
    //}
    //result_msg.img = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    publish_annotated_image(img, result_msg);
    pub.publish(result_msg);
    cnt++;
    
    if (test_speed) cout << "Time: " << chrono::duration_cast<chrono::nanoseconds>(end - start).count() / 1e9 << "s" << endl;
}

void aimImage(Mat &img)
{
    auto start = chrono::high_resolution_clock::now();
    char color = 'b';
    msgs::Armors result_msg;

    //resize(img, img, Size(img_width, img_height));
    vector<Armor> result = Detector::analyze(img, color);

    msgs::ArmorItem item;
    //geometry_msgs::Vector3 result_msg;
    
    result_msg.count = result.size();
    result_msg.items.clear();
    
    for (size_t i = 0; i < result.size(); i++)
    {
        item.x1 = max((int)result[i].upper_l.x, 0);
        item.y1 = max((int)result[i].upper_l.y, 0);
        item.x2 = max((int)result[i].lower_l.x, 0);
        item.y2 = max((int)result[i].lower_l.y, 0);
        item.x3 = max((int)result[i].lower_r.x, 0);
        item.y3 = max((int)result[i].lower_r.y, 0);
        item.x4 = max((int)result[i].upper_r.x, 0);
        item.y4 = max((int)result[i].upper_r.y, 0);
        item.area = (int)result[i].area();
        item.id = 0; // to be changed with identification
        result_msg.items.push_back(item);
    }
    auto end = chrono::high_resolution_clock::now();
    cout << "Published result of " << result_msg.count << " armors" << endl;
    //if (result_msg.count > 0)
    //{
    //    cout << item.cx << " " << item.cy << endl;
    //}
    //result_msg.img = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    publish_annotated_image(img, result_msg);
    pub.publish(result_msg);
    cnt++;
    
    if (test_speed) cout << "Time: " << chrono::duration_cast<chrono::nanoseconds>(end - start).count() / 1e9 << "s" << endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detect_and_publish");
    
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    
    pub = n.advertise<msgs::Armors>("armors", 10);
    image_pub = it.advertise("annotated_image", 10);
    
    Mat src;
    camera::Camera image_capture;
    int cnt = 0;
    while (ros::ok())
    {
        image_capture.ReadImg(src);
        if (src.empty())
            continue;
        aimImage(src);
        ros::spinOnce();
    }
    
    ros::spin();

    return 0;
}

    /*
    vector<Mat> imgs;
    Mat img;
    vector<Armor> result;
    Detector detector;
    string file_path;
    int num = start_num, sum = 0, found = 0;
    char color = 'r';

    //auto start = chrono::high_resolution_clock::now();
    for (size_t i = 0; i < imgs.size(); i++)
    {
        result = detector.analyze(imgs[i], color);
        sum += result.size();
        found += (result.size() > 0);
        //cout << sum << endl;
    }
    //double end = clock();
    //cout << (end - start) / CLOCKS_PER_SEC << endl;
    //auto end = chrono::high_resolution_clock::now();
    
    cout << "In " << picture_count << " pictures" << endl;
    cout << "Pictures containing armor: " << found << endl;
    cout << "Total number of armors found: " << sum << endl;
    if (test_speed) cout << "Time elapsed: " << chrono::duration_cast<chrono::nanoseconds>(end - start).count() / 1e9 << "s" << endl;
    */
