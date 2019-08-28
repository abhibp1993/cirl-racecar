#pragma once
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <string>

class PathGenerator {
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;
    nav_msgs::Path path_;
    std::string topic_;

    void add_to_path(const geometry_msgs::PoseWithCovarianceStamped&);

    public:
        PathGenerator(std::string);
        void publish();
};
