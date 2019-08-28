#include <racecar_tools/path_generator.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/console.h>

PathGenerator::PathGenerator(std::string topic) {
    topic_ = topic;
    subscriber_ = nh_.subscribe(topic, 1000, &PathGenerator::add_to_path, this);
    publisher_ = nh_.advertise<nav_msgs::Path>("/path" + topic, 1);
}

void PathGenerator::add_to_path(const geometry_msgs::PoseWithCovarianceStamped& msg) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = msg.header.stamp;
    pose_stamped.header.frame_id = msg.header.frame_id;
    pose_stamped.pose = msg.pose.pose;
    path_.header.stamp = ros::Time::now();
    path_.header.frame_id = msg.header.frame_id;
    path_.poses.push_back(pose_stamped);
}

void PathGenerator::publish() {
    if (path_.header.stamp.sec > 0) {
        publisher_.publish(path_);
    }
}
