#pragma once

#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace imuController{
    double roll, pitch, yaw;
    void imuCallback(const sensor_msgs::Imu msg){
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        tfBuffer.lookupTransform("imu_link", "imu", ros::Time(0));
        tf2::Quaternion q_tf;
        tf2::convert(msg.orientation, q_tf);
        tf2::Matrix3x3 m(q_tf);
        m.getRPY(roll, pitch, yaw);
    }
}
