#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Vector3.h>
//#include <msgs/Armors.h>
//#include <msgs/ArmorItem.h>
//#include <msgs/Attack_cmd.h>
//#include <number_recognition/img_msg.h>
#include <std_msgs/Bool.h>
#include "armor.h"
#include "plain_detect.h"
#include "tools.h"
#include "hikrobot_camera.hpp"
#include "auto_aiming.h"
using namespace cv;
using namespace std;

// Camera parameters: /opt/MVS/Samples/64/Python/ParametrizeCamera_LoadAndSave
// New camera parameters: ~/MVS

// No change gamma: exposure time 2000 us
// Change gamma: exposure time 5000 us

/*
Modes:
0 - Velocity
1 - Acceleration (Deprecated)
2 - Idle
*/

int aiming_cnt = 0;

AutoAiming::AutoAiming()
{
    tar_id = 0;
    manual_label = 0;
    center = Point3f(0, 0, 0);
    last_act = Point2d(0, 0);
    last_tar = Armor();
    last_time = chrono::high_resolution_clock::now();
    cooldown = 0;
    //x_on_target = false;
    //mode = 1; // acceleration
    missed_tar = 0;
    cv::FileStorage fs("/home/astar/catkin_ws/src/detection_old/src/params/camera_old_8mm.yml", cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_mtx;
    if (camera_mtx.empty())
    {
        std::cerr << "Failed to read camera matrix from file." << std::endl;
        return;
    }
    fs["distortion_coefficients"] >> camera_dist;
    if (camera_dist.empty())
    {
        std::cerr << "Failed to read distortion coefficients from file." << std::endl;
        return;
    }
}

void AutoAiming::matchArmors(Mat &img, vector<Armor> &armors)
{
    int max_area = 0;
    vector<bool> get_corresponding_armor(last_armors.size());
    if (armors.size() == 0)
        return;
    sort(armors.begin(), armors.end(), Armor::cmp);
    for (size_t i = 0; i < armors.size(); i++)
    {
        bool found_last_armor = false;
        //cout << armors[i].center.x << " " << armors[i].center.y << " " << armors[i].area() << endl; 
        if (last_armors.size() > 0)
            for (size_t j = 0; j < last_armors.size(); j++)
                if (!get_corresponding_armor[j] && point_distance(last_armors[j].center, armors[i].center) <= MAX_TARGET_FRAME_DISTANCE)
                {
                    //cout << "Using last id: " << last_armors[j].id << endl;
                    get_corresponding_armor[j] = true;
                    armors[i].id = last_armors[j].id;
                    found_last_armor = true;
                    break;
                }
        if (!found_last_armor)
        {
            manual_label++;
            armors[i].id = manual_label; // 0
            new_armors.emplace_back(armors[i]);
            new_armors.back().id = manual_label; // i
        }
    }
/*
    for (int i = 0; i < new_armors.size(); i++)
    {
        int res_id = 0;// identifyArmor(img, new_armors[i]);
        armors[new_armors[i].id] = new_armors[i];
        armors[new_armors[i].id].id = res_id;
        new_armors[i].id = res_id;
    }
*/ // Indentify armor not used
}

Armor AutoAiming::pickTarget(vector<Armor> &armors)
{
    if (armors.size() == 0)
        return Armor();
    for (int i = 0; i < armors.size(); i++)
        if (armors[i].id == tar_id && armors[i].area() >= MIN_TARGET_AREA)
            return armors[i];
    /*
    if (last_tar.center.x != 0 && last_tar.center.y != 0)
    {
        int last_tar_index = distance(dists.begin(), min_element(dists.begin(), dists.end()));
        if (armors[last_tar_index].area() >= MIN_TARGET_AREA)
            tar = armors[last_tar_index];
    }
    */
    
    if (armors.size() > 0)
    {
        tar_id = armors[0].id;
        return armors[0];
    }
    tar_id = 0;
    
    return Armor();
}

void AutoAiming::traceTarget(Armor tar)
{
    /*
    if (tar.empty())
    {
        missed_tar++;
        if (missed_tar >= 50 && mode != 2)
        {
            mode = 2;
            idle_angle = 0;
        }
    }

    if (mode == 2)
    {
        if (idle_angle < 0.4)
            controller->moveGimbal(0, 0.01);
        else
            controller->moveGimbal(0, -0.01);
        if (idle_angle >= 0.8)
            idle_angle = 0;
        return;
    }
    */

    if (tar.empty())
        return;

    missed_tar = 0;
    
    //ros::Time time0 = ros::Time::now();
    cv::Mat p_img = (cv::Mat_<float>(4, 2) << tar.upper_l.x, tar.upper_l.y, tar.lower_l.x, tar.lower_l.y, tar.lower_r.x, tar.lower_r.y, tar.upper_r.x, tar.upper_r.y);
    //std::cout << "Got armor position at: " << p_img.row(0) << " " << p_img.row(1) << " " << p_img.row(2) << " " << p_img.row(3) << std::endl;
    cv::Mat rvec, tvec;
    pixel_to_cam(p_img, camera_mtx, camera_dist, rvec, tvec);
    tvec.convertTo(tvec, CV_32F);
    //std::cout << "Position in camera coordinate system: " << tvec.at<double>(0) << " " << tvec.at<double>(1) << " " << tvec.at<double>(2) << std::endl;
    cv::Mat predicted_tvec = predictor.predict(tvec).rowRange(0, 3);
    if (aiming_cnt % 100 == 0)
        std::cout << "Predicted position: " << predicted_tvec.at<float>(0) << " " << predicted_tvec.at<float>(1) << " " << predicted_tvec.at<float>(2) << std::endl;
    if (predicted_tvec.at<float>(2) == 0)
        return;
    predicted_tvec -= cv::Mat(center);
    //float yaw_angle = std::atan(predicted_tvec.at<float>(0) / predicted_tvec.at<float>(2));
    //float pitch_angle = std::atan(predicted_tvec.at<float>(1) / predicted_tvec.at<float>(2));
    float yaw_angle = std::atan(tvec.at<float>(0) / tvec.at<float>(2));
    float pitch_angle = std::atan(tvec.at<float>(1) / tvec.at<float>(2));
    if (aiming_cnt % 100 == 0)
    {
        std::cout << "Yaw: " << abs(yaw_angle) / CV_PI * 180 << " degrees ";
        std::cout << (yaw_angle > 0 ? "right" : "left") << ", Pitch: " << abs(pitch_angle) / CV_PI * 180 << " degrees ";
        std::cout << (pitch_angle > 0 ? "down" : "up") << std::endl;
    }
    // For debugging
    
    if (cooldown <= 0)
    {
        if (abs(yaw_angle) / CV_PI * 180 > 10)
    	    cout << "High yaw speed alert: " << abs(yaw_angle) / CV_PI * 180 << endl;
    	if (abs(pitch_angle) / CV_PI * 180 > 5)
    	    cout << "High pitch speed alert: " << abs(pitch_angle) / CV_PI * 180 << endl;
        controller->moveGimbal(-yaw_angle, pitch_angle);
        cooldown += max(abs(yaw_angle), abs(pitch_angle * 2)) / CV_PI * 180 / ROTATING_SPEED + 0.005;
    }
    //ros::Time time1 = ros::Time::now();
    //std::cout << (time1 - time0).toSec() << std::endl;
}

void AutoAiming::shootTarget(Armor tar)
{
    if (tar.empty())
        return;
    //if (SHOOT_X > tar.upper_l.x && SHOOT_X < tar.lower_r.x && 
    //    SHOOT_Y > tar.upper_l.y && SHOOT_Y < tar.lower_r.y)
    controller->shoot(1);
}

void AutoAiming::publishAnnotatedImage(Mat img, vector<Armor> armors)
{
    int aim_point_x = 717, aim_point_y = 501;
    line(img, Point(aim_point_x - 20, aim_point_y), Point(aim_point_x + 20, aim_point_y), Scalar(0, 255, 0), 3);
    line(img, Point(aim_point_x, aim_point_y - 20), Point(aim_point_x, aim_point_y + 20), Scalar(0, 255, 0), 3); 
    for (Armor armor: armors)
    {
        armor.upper_l.x = max((int)armor.upper_l.x, 0);
        armor.upper_l.y = max((int)armor.upper_l.y, 0);
        armor.lower_l.x = max((int)armor.lower_l.x, 0);
        armor.lower_l.y = min((int)armor.lower_l.y, IMG_H);
        armor.lower_r.x = min((int)armor.lower_r.x, IMG_W);
        armor.lower_r.y = min((int)armor.lower_r.y, IMG_H);
        armor.upper_r.x = min((int)armor.upper_r.x, IMG_W);
        armor.upper_r.y = max((int)armor.upper_r.y, 0);
        vector<Point2i> armor_points = {armor.upper_l, armor.lower_l, armor.lower_r, armor.upper_r};
        polylines(img, armor_points, true, Scalar(0, 255, 0), 3);
    }
    auto img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    image_pub.publish(img_msg);
}

bool AutoAiming::needPublish()
{
    if (last_armors.size() > 0 && last_tar.empty() || new_armors.size() > 0)
        return true;
    return false;
}

void AutoAiming::aim(Mat &img, char color)
{
    new_armors.clear();

    //auto time0 = chrono::high_resolution_clock::now();

    vector<Armor> armors = Detector::analyze(img, color);

    //auto time1 = chrono::high_resolution_clock::now();
    //double duration = chrono::duration_cast<chrono::nanoseconds>(time1 - time0).count() / 1e9;
    //cout << "Detection: " << duration << " seconds." << endl;

    //time0 = chrono::high_resolution_clock::now();

    matchArmors(img, armors);

    Armor target = pickTarget(armors);

    traceTarget(target);

    shootTarget(target);

    publishAnnotatedImage(img, armors); // for debugging

    if (!target.empty() && target.id != last_tar.id /* && aiming_cnt % 100 == 0*/)
        cout << "Get target of color " << color << ": Position " << target.center.x << " " << target.center.y 
            << " ID: " << target.id << endl;

    last_tar = target;
    last_act = act;
    last_armors = armors;
    auto current_time = chrono::high_resolution_clock::now();
    if (cooldown > 0)
        cooldown = max(0., cooldown - chrono::duration_cast<chrono::nanoseconds>(current_time - last_time).count() / 1e9);
    last_time = current_time;
    //cout << cooldown << endl;
    //time1 = chrono::high_resolution_clock::now();
    //duration = chrono::duration_cast<chrono::nanoseconds>(time1 - time0).count() / 1e9;
    //cout << "Aiming: " << duration << " seconds." << endl;
    //cout << fixed << setprecision(9) << time1.time_since_epoch().count() / (1e9) << endl;
}

/*
unsigned int AutoAiming::identifyArmor(Mat &img, Armor new_armor)
{
    cout << new_armor.upper_l.x << " " << new_armor.upper_l.y << "  " << new_armor.lower_r.x << " " << new_armor.lower_r.y << endl;
    
    if (new_armor.upper_l.x < 0)
        new_armor.upper_l.x = 0;
    if (new_armor.upper_l.y < 0)
        new_armor.upper_l.y = 0;
    if (new_armor.lower_r.x >= img.size().width)
        new_armor.lower_r.x = img.size().width - 1;
    if (new_armor.lower_r.y >= img.size().height)
        new_armor.lower_r.y = img.size().height - 1;
    if (new_armor.upper_l.x > new_armor.lower_r.x || new_armor.upper_l.y > new_armor.lower_r.y)
        return 0;
    Mat cropped_img = img(Range(new_armor.upper_l.y, new_armor.lower_r.y), Range(new_armor.upper_l.x, new_armor.lower_r.x));
    std_msgs::Header header;
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_message;
    header.seq = 1;
    header.stamp = ros::Time::now();
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, cropped_img);
    img_bridge.toImageMsg(img_message);
    number_recognition::img_msg srv;
    srv.request.image = img_message;
    if (recognition_client.call(srv))
        cout << "Recognition service call successful." << endl;
    else
        cout << "Recognition service call failed" << endl;
    return srv.response.num;
}
*/

/*
void AutoAiming::publishArmor()
{
    //Mat cropped_img = img(Range(armor.upper_l.x, armor.lower_r.x), Range(armor.upper_l.y, armor.lower_r.y));

    msgs::Armors out_msg;
    //out_msg.header = msg->header;
    for (int i = 0; i < last_armors.size(); i++)
    {
        msgs::ArmorItem item;
        
        item.x1 = max((int)last_armors[i].upper_l.x, 0);
        item.y1 = max((int)last_armors[i].upper_l.y, 0);
        item.x2 = max((int)last_armors[i].lower_l.x, 0);
        item.y2 = min((int)last_armors[i].lower_l.y, IMG_H);
        item.x3 = min((int)last_armors[i].lower_r.x, IMG_W);
        item.y3 = min((int)last_armors[i].lower_r.y, IMG_H);
        item.x4 = min((int)last_armors[i].upper_r.x, IMG_W);
        item.y4 = max((int)last_armors[i].upper_r.y, 0);
        item.area = last_armors[i].area();
        item.id = last_armors[i].id;
        //cout << "Armor id: " << item.id << endl;
        //if (item.id == 0)
        //    continue;
        out_msg.items.emplace_back(item);
    }
    out_msg.count = out_msg.items.size();
    armor_pub.publish(out_msg);
}
*/
void AutoAiming::publishStatus()
{
    std_msgs::Bool status;
    if (last_tar.empty())
        status.data = false;
    else
        status.data = true;
    armor_pub.publish(status);
}

void AutoAiming::adjustCenter(Point3f adjustment)
{
    center += adjustment;
    std::cout << "Received adjustment: " << adjustment.x << " " << adjustment.y << " " << adjustment.z << std::endl;
    std::cout << "Current center: " << center << std::endl;
}

AutoAiming aim_object;
auto time_start = chrono::high_resolution_clock::now();

void aimImage(Mat &img)
{
    auto start = chrono::high_resolution_clock::now();
    char color = 'b';
    
    if (aiming_cnt == 0)
        time_start = chrono::high_resolution_clock::now();

    auto time0 = chrono::high_resolution_clock::now();
    Mat gamma_changed_img = img; //changeGamma(img, IMAGE_GAMMA);
    
    //aim_object.img_pub.publish(cv_ptr->toImageMsg());
    aiming_cnt++;
    
    aim_object.aim(gamma_changed_img, color);
    
    /*
    if (aiming_cnt % 20 == 0 || aim_object.needPublish())
    {
        aim_object.publishStatus(); // Only used in RMUL2023
        cout << "Published armor" << endl;
    }
    */
    
    auto time1 = chrono::high_resolution_clock::now();
    auto duration0 = chrono::duration_cast<chrono::nanoseconds>(time1 - time0).count() / 1e9;
    if (aiming_cnt % 100 == 0)
    {
        cout << "Duration: " << duration0 << endl;
        auto time_end = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::nanoseconds>(time_end - time_start).count() / 1e9;
        cout << "Aiming count: " << aiming_cnt << " Time elapsed: " << fixed << setprecision(9) << duration << " seconds." << endl;
    }
    //resize(img, img, Size(img_width, img_height));
    //result_msg.img = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
}

void callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);

    Mat img = cv_ptr->image;

    aimImage(img);
}

void adjustment_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    aim_object.adjustCenter(Point3f(msg->x, msg->y, msg->z));
}

Mat changeGamma(Mat &img, double gamma)
{
    Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    for( int i = 0; i < 256; ++i)
        p[i] = saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
    Mat res = img.clone();
    LUT(img, lookUpTable, res);
    return res;
}

/*
void targetCallback(msgs::Attack_cmd cmd)
{
    aim_object.tar_id = cmd.target_id;
}
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_aiming");
    ros::NodeHandle n;

    //aim_object.img_pub = n.advertise<sensor_msgs::Image>("cropped_armor_images", 10);
    aim_object.armor_pub = n.advertise<std_msgs::Bool>("armor_in_sight", 10);
    image_transport::ImageTransport it(n);
    aim_object.image_pub = it.advertise("annotated_image", 10);
    aim_object.controller = new Controller();
    //aim_object.recognition_client = n.serviceClient<number_recognition::img_msg>("Recognition");
    //ros::Subscriber target_sub = n.subscribe("decision/attack", 10, targetCallback);
    //ros::Subscriber game_start_sub = n.subscribe()
    ros::Subscriber sub_adjustment = n.subscribe("auto_aiming_adjustment", 10, adjustment_callback);
    if (READ_IMAGE_FROM_TOPIC)
    {
        ros::Subscriber sub = n.subscribe("detection/img", 10, callback);
        ros::spin();
    }
    else
    {
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
    }

    return 0;
}
