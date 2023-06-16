#ifndef TRACK__TRACKER_NODE_CPP_
#define TRACK__TRACKER_NODE_CPP_

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// #include <visualization_msgs/MarkerArray.h>


#include <roborts_msgs/GimbalAngle.h>
#include <roborts_msgs/ShootCmd.h>

#include "geometry_msgs/PoseStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tracker.hpp"
#include "robot.hpp"
#include "extended_kalman_filter.hpp"

#include "my_msgs/Armor.h"
#include "my_msgs/Armors.h"
#include "my_msgs/Target.h"
#include "my_msgs/TrackerInfo.h"


ros::NodeHandle* nh;
ros::Publisher info_pub_;
ros::Publisher target_pub_;
ros::Publisher marker_pub_;
ros::Subscriber armors_sub_;

// Maximum allowable armor distance in the XOY plane
double max_armor_distance_;

// The time when the last message was received
ros::Time last_time_;
double dt_;

// Armor tracker
double s2qxyz_, s2qyaw_, s2qr_;
double r_xyz_factor, r_yaw;
double lost_time_thres_;
std::unique_ptr<Tracker> tracker_;

// Reset tracker service
ros::ServiceServer reset_tracker_srv_;

Controller* Robot;

tf2_ros::Buffer buffer;
tf2_ros::TransformListener *listener;

// Visualization marker publisher
visualization_msgs::Marker position_marker_;
visualization_msgs::Marker linear_v_marker_;
visualization_msgs::Marker angular_v_marker_;
visualization_msgs::Marker armor_marker_;


void publishMarkers(const my_msgs::Target & target_msg)
{
    position_marker_.header = target_msg.header;
    linear_v_marker_.header = target_msg.header;
    angular_v_marker_.header = target_msg.header;
    armor_marker_.header = target_msg.header;
    
    visualization_msgs::MarkerArray marker_array;

    if (target_msg.tracking)
    {
        double yaw = target_msg.yaw, r1 = target_msg.radius_1, r2 = target_msg.radius_2;
        double xc = target_msg.position.x, yc = target_msg.position.y, za = target_msg.position.z;
        double vx = target_msg.velocity.x, vy = target_msg.velocity.y, vz = target_msg.velocity.z;
        double dz = target_msg.dz;
        ROS_INFO("Yaw:%.2f r1:%.2f, r2:%.2f (%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f)", yaw, r1, r2, xc, yc, za, vx, vy, vz);

        position_marker_.action = visualization_msgs::Marker::ADD;
        position_marker_.pose.position.x = xc;
        position_marker_.pose.position.y = yc;
        position_marker_.pose.position.z = za + dz / 2;

        linear_v_marker_.action = visualization_msgs::Marker::ADD;
        linear_v_marker_.points.clear();
        linear_v_marker_.points.emplace_back(position_marker_.pose.position);
        geometry_msgs::Point arrow_end = position_marker_.pose.position;
        arrow_end.x += vx;
        arrow_end.y += vy;
        arrow_end.z += vz;
        linear_v_marker_.points.emplace_back(arrow_end);

        angular_v_marker_.action = visualization_msgs::Marker::ADD;
        angular_v_marker_.points.clear();
        angular_v_marker_.points.emplace_back(position_marker_.pose.position);
        arrow_end = position_marker_.pose.position;
        arrow_end.z += target_msg.v_yaw / M_PI;
        angular_v_marker_.points.emplace_back(arrow_end);

        armor_marker_.action = visualization_msgs::Marker::ADD;
        armor_marker_.scale.y = tracker_->tracked_armor.type == "small" ? 0.135 : 0.23;
        bool is_current_pair = true;
        
        size_t a_n = target_msg.armors_num;
        geometry_msgs::Point p_a;
        double r = 0;
        for (size_t i = 0; i < a_n; i++)
        {
            double tmp_yaw = yaw + i * (2 * M_PI / a_n);
            // Only 4 armors has 2 radius and height
            if (a_n == 4)
            {
                r = is_current_pair ? r1 : r2;
                p_a.z = za + (is_current_pair ? 0 : dz);
                is_current_pair = !is_current_pair;
            }
            else
            {
                r = r1;
                p_a.z = za;
            }
            p_a.x = xc - r * cos(tmp_yaw);
            p_a.y = yc - r * sin(tmp_yaw);

            armor_marker_.id = i;
            armor_marker_.pose.position = p_a;
            tf2::Quaternion q;
            q.setRPY(0, target_msg.id == "outpost" ? -0.26 : 0.26, tmp_yaw);
            // armor_marker_.pose.orientation = tf2::toMsg(q);
            marker_array.markers.emplace_back(armor_marker_);
        }
    }
    else
    {
        position_marker_.action = visualization_msgs::Marker::DELETE;
        linear_v_marker_.action = visualization_msgs::Marker::DELETE;
        angular_v_marker_.action = visualization_msgs::Marker::DELETE;

        armor_marker_.action = visualization_msgs::Marker::DELETE;
        marker_array.markers.emplace_back(armor_marker_);
    }

    marker_array.markers.emplace_back(position_marker_);
    marker_array.markers.emplace_back(linear_v_marker_);
    marker_array.markers.emplace_back(angular_v_marker_);
    marker_pub_.publish(marker_array);
    ROS_INFO("marker published");
}

class Timer{
private:
    clock_t last_time;
public:
    Timer():last_time(clock()){}
    double get(){
        double res = (double)(clock()-last_time)/CLOCKS_PER_SEC;
        last_time = clock();
        return res;
    }
}T;

void armorsCallback(const my_msgs::Armors::ConstPtr& armors_msg_)
{
    ROS_INFO("--------------------------- Time per callback: %lf", T.get());
    ROS_INFO("--------------------------- %d input", armors_msg_->armors.size());
    my_msgs::Armors armors_msg;
    armors_msg.header = armors_msg_->header;
    //transform the camera frame robot frame
    
    for (const my_msgs::Armor & armor_ : armors_msg_->armors)
    {
        geometry_msgs::PoseStamped pose_optical;
        geometry_msgs::PoseStamped pose_camera;
        geometry_msgs::PoseStamped pose_robot;
        pose_optical.header = armors_msg.header;
        pose_optical.pose = armor_.pose;
        try
        {

            pose_camera = buffer.transform(pose_optical,"camera");
            pose_robot = buffer.transform(pose_camera,"robot");

            double roll, pitch, yaw;
            tf::Quaternion tf_q;
            tf::quaternionMsgToTF(pose_robot.pose.orientation, tf_q);
            tf::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
                
            ROS_INFO("robot frame: (%.2f,%.2f,%.2f) roll: %.2f pitch: %.2f yaw: %.2f",pose_robot.pose.position.x,pose_robot.pose.position.y,pose_robot.pose.position.z, roll, pitch, yaw);
            my_msgs::Armor armor = armor_;
            armor.pose = pose_robot.pose;
            // Filter abnormal armors
            if (abs(armor.pose.position.z < 1.2 && Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm() < max_armor_distance_))
            {
                armors_msg.armors.emplace_back(armor);
            }
        }
        catch(const std::exception& e)
        {
            ROS_ERROR("Failed to transform armor pose: %s", e.what());
        }
    }
    ROS_INFO("Number of Armor: %d", armors_msg.armors.size());
    // Init message
    my_msgs::TrackerInfo info_msg;
    my_msgs::Target target_msg;
    ros::Time time = armors_msg.header.stamp;
    target_msg.header.stamp = time;
    target_msg.header.frame_id = "robot";
    
    // Update tracker
    if (tracker_->tracker_state == Tracker::LOST)
    {
        ROS_INFO("LOST");
        tracker_->init(armors_msg);
        target_msg.tracking = false;
    }
    else
    {
        dt_ = (time - last_time_).toSec();
        tracker_->lost_thres = static_cast<int>(lost_time_thres_ / dt_);
        tracker_->update(armors_msg);

        // Publish Info
        info_msg.position_diff = tracker_->info_position_diff;
        info_msg.yaw_diff = tracker_->info_yaw_diff;
        info_msg.position.x = tracker_->measurement(0);
        info_msg.position.y = tracker_->measurement(1);
        info_msg.position.z = tracker_->measurement(2);
        info_msg.yaw = tracker_->measurement(3);
        info_pub_.publish(info_msg);

        if (tracker_->tracker_state == Tracker::DETECTING)
        {
            target_msg.tracking = false;
            ROS_INFO("DETECTING");
        }
        else if (tracker_->tracker_state == Tracker::TRACKING ||
                 tracker_->tracker_state == Tracker::TEMP_LOST)
        {
            if (tracker_->tracker_state == Tracker::TRACKING)
                ROS_INFO("TRACKING");
            if (tracker_->tracker_state == Tracker::TEMP_LOST)
                ROS_INFO("TEMP_LOST");
            target_msg.tracking = true;
            // Fill target message
            const auto & state = tracker_->target_state;
            target_msg.id = tracker_->tracked_id;
            target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num);
            target_msg.position.x = state(0);
            target_msg.velocity.x = state(1);
            target_msg.position.y = state(2);
            target_msg.velocity.y = state(3);
            target_msg.position.z = state(4);
            target_msg.velocity.z = state(5);
            target_msg.yaw = state(6);
            target_msg.v_yaw = state(7);
            target_msg.radius_1 = state(8);
            target_msg.radius_2 = tracker_->another_r;
            target_msg.dz = tracker_->dz;
        }
    }
    last_time_ = time;

    target_pub_.publish(target_msg);
    publishMarkers(target_msg);
}

void init_EKF()
{
    // left pos right neg
    // EKF
    // xa = x_armor, xc = x_robot_center
    //        0 , 1   , 2 , 3   , 4 , 5   , 6  , 7    , 8
    // state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r
    // measurement: xa, ya, za, yaw
    // f - Process function
    auto f = [](const Eigen::VectorXd & x)
    {
        Eigen::VectorXd x_new = x;
        x_new(0) += x(1) * dt_;
        x_new(2) += x(3) * dt_;
        x_new(4) += x(5) * dt_;
        x_new(6) += x(7) * dt_;
        return x_new;
    };
    // J_f - Jacobian of process function
    auto j_f = [](const Eigen::VectorXd & x)
    {
        Eigen::MatrixXd f(9, 9);
        // clang-format off
        f <<  1,   dt_, 0,   0,   0,   0,   0,   0,   0,
            0,   1,   0,   0,   0,   0,   0,   0,   0,
            0,   0,   1,   dt_, 0,   0,   0,   0,   0, 
            0,   0,   0,   1,   0,   0,   0,   0,   0,
            0,   0,   0,   0,   1,   dt_, 0,   0,   0,
            0,   0,   0,   0,   0,   1,   0,   0,   0,
            0,   0,   0,   0,   0,   0,   1,   dt_, 0,
            0,   0,   0,   0,   0,   0,   0,   1,   0,
            0,   0,   0,   0,   0,   0,   0,   0,   1;
        // clang-format on
        return f;
    };
    // h - Observation function
    auto h = [](const Eigen::VectorXd & x)
    {
        Eigen::VectorXd z(4);
        double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
        z(0) = xc - r * cos(yaw);  // xa
        z(1) = yc - r * sin(yaw);  // ya
        z(2) = x(4);               // za
        z(3) = x(6);               // yaw
        return z;
    };
    // J_h - Jacobian of observation function
    auto j_h = [](const Eigen::VectorXd & x)
    {
        Eigen::MatrixXd h(4, 9);
        double yaw = x(6), r = x(8);
        // clang-format off
        //    xc   v_xc yc   v_yc za   v_za yaw         v_yaw r
        h <<  1,   0,   0,   0,   0,   0,   r*sin(yaw), 0,   -cos(yaw),
            0,   0,   1,   0,   0,   0,   -r*cos(yaw),0,   -sin(yaw),
            0,   0,   0,   0,   1,   0,   0,          0,   0,
            0,   0,   0,   0,   0,   0,   1,          0,   0;
        // clang-format on
        return h;
    };
    // update_Q - process noise covariance matrix
    s2qxyz_ = nh->param<double>("ekf_sigma2_q_xyz", 20.0);
    s2qyaw_ = nh->param<double>("ekf_sigma2_q_yaw", 100.0);
    s2qr_ = nh->param<double>("ekf_sigma2_q_r", 800.0);
    auto u_q = []()
    {
        Eigen::MatrixXd q(9, 9);
        double t = dt_, x = s2qxyz_, y = s2qyaw_, r = s2qr_;
        double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
        double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
        double q_r = pow(t, 4) / 4 * r;
        // clang-format off
        //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
        q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,
            q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,
            0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,
            0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,
            0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,
            0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,
            0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,
            0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,
            0,      0,      0,      0,      0,      0,      0,      0,      q_r;
        // clang-format on
        return q;
    };
    // update_R - measurement noise covariance matrix
    r_xyz_factor = nh->param<double>("ekf_r_xyz_factor", 0.05);
    r_yaw = nh->param<double>("ekf_r_yaw", 0.02);
    auto u_r = [](const Eigen::VectorXd & z)
    {
        Eigen::DiagonalMatrix<double, 4> r;
        double x = r_xyz_factor;
        r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_yaw;
        return r;
    };
    // P - error estimate covariance matrix
    Eigen::DiagonalMatrix<double, 9> p0;
    p0.setIdentity();
    tracker_->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};
}

bool doReq(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
    tracker_->tracker_state = Tracker::LOST;
    resp.success = true;
    ROS_INFO("Tracker reset!");
    return true;
}

int main(int argc, char *argv[])
{
    //initiate a new node
    ros::init(argc, argv, "tracker_node");

    nh = new ros::NodeHandle();
    // Robot = new Controller;
    ROS_INFO("Starting TrackerNode!");
    //yaw_angle, pitch_angle, pitch_offset, absolute

    listener = new tf2_ros::TransformListener(buffer);
    // while (ros::ok())
    // {
    //     Robot->moveGimbal(0.02, 0.00, 0.00);
    //     ros::spinOnce();
    // }


    // publish the robot_center frame
    // tf2_ros::StaticTransformBroadcaster pub;
    // geometry_msgs::TransformStamped tfs;
    // tfs.header.stamp = ros::Time::now();
    // tfs.header.frame_id = "robot_center";
    // tfs.header.frame_id = "";
    
    
    // Maximum allowable armor distance in the XOY plane
    max_armor_distance_ = nh->param<double>("max_armor_distance", 10.0);
    
    // Tracker
    double max_match_distance = nh->param<double>("tracker_max_match_distance", 0.5);
    double max_match_yaw_diff = nh->param<double>("tracker_max_match_yaw_diff", 1.0);
    tracker_ = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);
    
    tracker_->tracking_thres = nh->param<int>("tracker_tracking_thres", 5);
    lost_time_thres_ = nh->param<double>("tracker_lost_time_thres", 2);

    init_EKF();

    // Reset tracker service
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;

    reset_tracker_srv_ = nh->advertiseService("/tracker/reset", doReq);

    armors_sub_ = nh->subscribe("/detection/armors", 10, armorsCallback);
    info_pub_ = nh->advertise<my_msgs::TrackerInfo>("/tracker/info", 10);
    target_pub_ = nh->advertise<my_msgs::Target>("/tracker/target", 1);
    
    // Visualization Marker Publisher
    // See http://wiki.ros.org/rviz/DisplayTypes/Marker
    position_marker_.ns = "position";
    position_marker_.type = visualization_msgs::Marker::SPHERE;
    position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
    position_marker_.color.a = 1.0;
    position_marker_.color.g = 1.0;
    linear_v_marker_.type = visualization_msgs::Marker::ARROW;
    linear_v_marker_.ns = "linear_v";
    linear_v_marker_.scale.x = 0.03;
    linear_v_marker_.scale.y = 0.05;
    linear_v_marker_.color.a = 1.0;
    linear_v_marker_.color.r = 1.0;
    linear_v_marker_.color.g = 1.0;
    angular_v_marker_.type = visualization_msgs::Marker::ARROW;
    angular_v_marker_.ns = "angular_v";
    angular_v_marker_.scale.x = 0.03;
    angular_v_marker_.scale.y = 0.05;
    angular_v_marker_.color.a = 1.0;
    angular_v_marker_.color.b = 1.0;
    angular_v_marker_.color.g = 1.0;
    armor_marker_.ns = "armors";
    armor_marker_.type = visualization_msgs::Marker::CUBE;
    armor_marker_.scale.x = 0.03;
    armor_marker_.scale.z = 0.125;
    armor_marker_.color.a = 1.0;
    armor_marker_.color.r = 1.0;
    marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("/tracker/marker_array_topic", 10);
    ros::spin();
    
    return 0;
}

#endif