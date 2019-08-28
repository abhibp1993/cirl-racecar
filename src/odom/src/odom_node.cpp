#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <odom/odom_controller.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "odom_node");
    ros::NodeHandle nh;

    OdomController odom_controller;
    ros::Publisher odom_pub =
        nh.advertise<nav_msgs::Odometry>("/bicycle/odom", 1);
    ros::Publisher pose_pub = 
        nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/bicycle/pose", 1);

    ros::Rate loop_rate(1000);

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    while(ros::ok()){
        const nav_msgs::Odometry current_odom =
                odom_controller.getOdom();

        if (current_odom.header.stamp.sec > 0) {
            /* Odom Transform */
            transformStamped.header.stamp = current_odom.header.stamp;
            transformStamped.header.frame_id = "odom";
            transformStamped.child_frame_id = "base_link";
            transformStamped.transform.translation.x 
                    = current_odom.pose.pose.position.x;
            transformStamped.transform.translation.y 
                    = current_odom.pose.pose.position.y;
            transformStamped.transform.translation.z 
                    = current_odom.pose.pose.position.z;

            transformStamped.transform.rotation 
                    = current_odom.pose.pose.orientation;

            // br.sendTransform(transformStamped);

            const geometry_msgs::PoseWithCovarianceStamped current_pose =
                    odom_controller.getPose();
            pose_pub.publish(current_pose);
            odom_pub.publish(current_odom);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
