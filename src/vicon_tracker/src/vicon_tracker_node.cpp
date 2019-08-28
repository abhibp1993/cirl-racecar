#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Float64.h>

geometry_msgs::TransformStamped vicon_transform;
geometry_msgs::PoseStamped pose;
nav_msgs::Path path;
bool first_callback = true;

void vicon_callback(geometry_msgs::TransformStamped data){
    if(first_callback){
		// Since odom starts at (0, 0) the transform is just
		// the first value vicon sends
		vicon_transform.header.frame_id = "map";
		vicon_transform.child_frame_id = "odom";
		vicon_transform.transform = data.transform;
        first_callback = false;
    }
    const geometry_msgs::Vector3 trans = data.transform.translation;
    const geometry_msgs::Quaternion orient = data.transform.rotation;
    geometry_msgs::Point pos;
    pos.x = trans.x; pos.y = trans.y; pos.z = trans.z;

    pose.header.frame_id = "map";
    pose.pose.position = pos;
    pose.pose.orientation = orient;

    path.header.frame_id = "map";
    path.poses.push_back(pose);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "vicon_tracker_node");
    ros::NodeHandle nh;
    
    ros::Publisher path_pub =
        nh.advertise<nav_msgs::Path>("/vicon_tracker/path", 1);
    ros::Publisher pose_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/vicon_tracker/pose", 1);
    ros::Subscriber vicon_sub = 
            nh.subscribe("/vicon/racecar_mqp/racecar_mqp", 
            1, vicon_callback);

    static tf2_ros::TransformBroadcaster br;
    ros::Rate loop_rate(1000);

    while(ros::ok()){
		if(!first_callback){
            vicon_transform.header.stamp = ros::Time::now();
			br.sendTransform(vicon_transform);

            path.header.stamp = ros::Time::now();
			path_pub.publish(path);

            pose.header.stamp = ros::Time::now();
			pose_pub.publish(pose);
		}
        ros::spinOnce();
        loop_rate.sleep();
    }
}
