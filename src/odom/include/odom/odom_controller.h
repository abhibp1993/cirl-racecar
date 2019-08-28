#pragma once

#include <ros/ros.h>
#include <string>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class OdomController{

// Subscribers & Topics
ros::NodeHandle nh_;
ros::Subscriber steer_angle_sub_;
ros::Subscriber velocity_sub_;
const std::string steer_angle_topic_ = "/drive/steer";
const std::string velocity_topic_ = "/encoder/velocity";

// Constants & Variables
const std::string odom_frame_id_ = "odom";
const std::string odom_child_frame_id_ = "base_link";
static constexpr float wheelbase_ = 0.32; // b in bicycle model
static constexpr float CoM_to_rear_ = 0.16; //a in bicycle model
float CoM_angle_radian_ = 0.0;
float steer_angle_radian_ = 0.0;
float heading_yaw_ = 0.0;
double prev_pose_time_ = 0.0;
float CoM_velocity_ = 0.0;
geometry_msgs::Vector3Stamped velocity_;
nav_msgs::Odometry odom_;
geometry_msgs::PoseWithCovarianceStamped pose_;

// Methods
void updateSteerAngle(const std_msgs::UInt16& msg);
void updateVelocity(const geometry_msgs::Vector3Stamped& msg);
void updatePose();
void updateLinearTwist();
void updateAngularTwist();
void calculateOdom();

public:
    OdomController();
    nav_msgs::Odometry getOdom(){ return odom_; }
    float getSteerAngle(){ return steer_angle_radian_; }
    geometry_msgs::PoseWithCovarianceStamped getPose(){ return pose_; }
};

