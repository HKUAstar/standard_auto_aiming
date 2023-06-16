#include <opencv2/core.hpp>
#include <vector>
#include <iostream>
#include "armor.h"
#include "plain_detect.h"

bool test_speed = true, test_acc = false;

int cnt = 0;

double point_distance(cv::Point2f l, cv::Point2f r)
{
    return std::sqrt(std::pow(l.x - r.x, 2) + std::pow(l.y - r.y, 2));
}

cv::Mat Detector::separateColors(cv::Mat img, char color)
{
    std::vector<cv::Mat> channels;
    cv::split(img, channels);

    cv::Mat gray_img;

    if (color == 'r')
        gray_img = channels.at(2) - channels.at(0);
    else
        gray_img = channels.at(0) - channels.at(2);

    return gray_img;
}

cv::Mat Detector::binarization(cv::Mat img)
{
    cv::Mat bin_bright_img;

    cv::threshold(img, bin_bright_img, brightness_threshold, 255, cv::THRESH_BINARY);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));

    cv::dilate(bin_bright_img, bin_bright_img, element);
    //imshow("Binarization", bin_bright_img);
    //waitKey();

    return bin_bright_img;
}

std::vector<std::vector<cv::Point>> Detector::getContours(cv::Mat img)
{
    std::vector<std::vector<cv::Point>> light_contours;
    cv::findContours(img, light_contours, cv::RetrievalModes::RETR_EXTERNAL, cv::ContourApproximationModes::CHAIN_APPROX_SIMPLE);

    return light_contours;
}

cv::RotatedRect& Detector::adjustRec(cv::RotatedRect& rec)
{
    float& width = rec.size.width;
    float& height = rec.size.height;
    float& angle = rec.angle;

    while (angle >= 90.0) angle -= 180.0;
    while (angle < -90.0) angle += 180.0;

    if (angle >= 45.0)
    {
        std::swap(width, height);
        angle -= 90.0;
    }
    else if (angle < -45.0)
    {
            std::swap(width, height);
            angle += 90.0;
    }

    return rec;
}

bool Detector::compareLightIndicators(cv::RotatedRect l1, cv::RotatedRect l2)
{
    return l1.center.x < l2.center.x;
}

std::vector<std::vector<cv::Point>> Detector::filterContours(std::vector<std::vector<cv::Point>>& light_contours, std::vector<cv::RotatedRect>& light_info)
{
    std::vector<std::vector<cv::Point>> remaining_contours;
    for (const auto& contour:light_contours)
    {
        float light_contour_area = cv::contourArea(contour);
        if (light_contour_area < light_min_area)
            continue;
        if (contour.size() < 5)
            continue;
        cv::RotatedRect light_rec = cv::fitEllipse(contour);
        adjustRec(light_rec);
        if (light_rec.size.width / light_rec.size.height > light_max_ratio ||
            light_contour_area / light_rec.size.area() < light_contour_min_solidity)
                continue;
        light_rec.size.width *= light_color_detect_extend_ratio;
        light_rec.size.height *= light_color_detect_extend_ratio;
        
        light_info.push_back(cv::RotatedRect(light_rec));
        remaining_contours.push_back(contour);
    }
    return remaining_contours;
}

std::vector<Armor> Detector::matchArmor(std::vector<cv::RotatedRect>& light_info)
{
    std::vector<Armor> armors;

    sort(light_info.begin(), light_info.end(), compareLightIndicators);
    for (size_t i = 0; i < light_info.size(); i++)
    {
        const cv::RotatedRect& left = light_info[i];
        for (size_t j = i + 1; j < light_info.size(); j++)
        {
            const cv::RotatedRect& right = light_info[j];
            
            double angle_diff = abs(left.angle - right.angle);
            double len_diff_ratio = abs(left.size.height - right.size.height) / std::max(left.size.height, right.size.height);
            
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

            int type = 0;
            if (dis_ratio > armor_small_max_aspect_ratio)
                type = 1;
            //std::cout << "Aspect ratio: " << dis_ratio << " Type: " << type << std::endl;

            Armor armor(left, right, type);
            armors.push_back(armor);
        }
    }
    return armors;
}

void Detector::showArmor(cv::Mat img, std::vector<Armor> armors)
{
    for (size_t i = 0; i < armors.size(); i++)
    {
        circle(img, armors[i].center, 1, cv::Scalar(0, 255, 0), 10);
        std::vector<cv::Point2i> armor_points = armors[i].coordinates();
        polylines(img, armor_points, true, cv::Scalar(0, 255, 0), 1);
    }
    cv::imshow("Armor", img);
    cv::waitKey();
}

/*
void Detector::drawAllContours(cv::Mat img, std::vector<std::vector<cv::Point>> light_contours)
{
    for (size_t i = 0; i < light_contours.size(); i++)
        cv::drawContours(img.clone(), light_contours, i, cv::Scalar(0, 0, 255), 1);
}
*/

std::vector<Armor> Detector::analyze(cv::Mat img, char color)
{
    //Mat debug_img = img.clone();
    img = separateColors(img, color);
    img = binarization(img);
    //Mat bin_img = img.clone();
    std::vector<std::vector<cv::Point>> contours = getContours(img);

    std::vector<cv::RotatedRect> light_info;
    contours = filterContours(contours, light_info);

    //drawAllContours(debug_img, contours);

    std::vector<Armor> armors = matchArmor(light_info);

    //cout << "Found " << armors.size() << " armor(s)" << endl;
    //if (test_acc) showArmor(debug_img, armors);
    
    return armors;
}

/*
void callback(const sensor_msgs::ImageConstPtr& msg)
{
    auto start = chrono::high_resolution_clock::now();
    char color = 'r';
    msgs::Armors result_msg;
    result_msg.header.stamp = ros::Time::now();
    
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
        item.cx = max((int)result[i].center.x, 0);
        item.cy = max((int)result[i].center.y, 0);
        item.x1 = max((int)result[i].upper_l.x, 0);
        item.y1 = max((int)result[i].upper_l.y, 0);
        item.x2 = max((int)result[i].lower_r.x, 0);
        item.y2 = max((int)result[i].lower_r.y, 0);
        item.area = (int)result[i].area();
        result_msg.items.push_back(item);
    }
    auto end = chrono::high_resolution_clock::now();
    //cout << "Published result of length " << result_msg.count << endl;
    if (result_msg.count > 0)
    {
        cout << item.cx << " " << item.cy << endl;
    }
    result_msg.img = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    
    //pub.publish(result_msg);
    cnt++;
    
    if (test_speed) cout << "Time: " << chrono::duration_cast<chrono::nanoseconds>(end - start).count() / 1e9 << "s" << endl;
}
*/

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
