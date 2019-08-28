#include <odom/odom_controller.h>
#include <math.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Vector3Stamped.h>

OdomController::OdomController(){
    steer_angle_sub_ = nh_.subscribe(steer_angle_topic_, 1000,
            &OdomController::updateSteerAngle, this);
    velocity_sub_ = nh_.subscribe(velocity_topic_, 1000,
            &OdomController::updateVelocity, this);
    odom_.header.frame_id = odom_frame_id_;
    odom_.child_frame_id = odom_child_frame_id_;

    // set diagonal covariance values
    for(int i = 0; i < 6; ++i){
        odom_.pose.covariance[i * 7] = 0.1;
        odom_.twist.covariance[i * 7] = 0.1;
    }
}

void OdomController::updateSteerAngle(const std_msgs::UInt16& msg){
    // msg.data > 1500 is left, we flip to be right
    steer_angle_radian_ = -((((msg.data - 1500.0) * 0.060) * M_PI) / 180.0);
    CoM_angle_radian_ =
            atan((CoM_to_rear_ / wheelbase_) * tan(steer_angle_radian_));
    CoM_velocity_ = velocity_.vector.x / cos(CoM_angle_radian_);
    calculateOdom();
}

void OdomController::updateVelocity(const geometry_msgs::Vector3Stamped& msg){
    velocity_ = msg;
    CoM_velocity_ = velocity_.vector.x / cos(CoM_angle_radian_);
    calculateOdom();
}

void OdomController::calculateOdom(){
    updatePose();
    updateLinearTwist();
    updateAngularTwist();
    odom_.header.stamp = velocity_.header.stamp;
}

void OdomController::updatePose(){
    const float velocity = velocity_.vector.x;
    const double delta_t = velocity_.header.stamp.toSec() -
            prev_pose_time_;
    prev_pose_time_ = velocity_.header.stamp.toSec();

    heading_yaw_ += (((velocity / wheelbase_)
                        * tan(steer_angle_radian_))
                        * delta_t);
    tf2::Quaternion orientation_quat;
    orientation_quat.setRPY( 0, 0, heading_yaw_ );
    orientation_quat.normalize();

    // Set Orientation part of Pose
    odom_.pose.pose.orientation = tf2::toMsg(orientation_quat);

    // Set relevant parts of Position part of Pose
    odom_.pose.pose.position.x += ((CoM_velocity_
                * cos(CoM_angle_radian_ + heading_yaw_))
                * delta_t);
    odom_.pose.pose.position.y += ((CoM_velocity_
                * sin(CoM_angle_radian_ + heading_yaw_))
                * delta_t);

    pose_.header.stamp = odom_.header.stamp;
    pose_.header.frame_id = odom_frame_id_;
    pose_.pose = odom_.pose;
}

void OdomController::updateLinearTwist(){
    odom_.twist.twist.linear.x = CoM_velocity_ * cos(heading_yaw_ + CoM_angle_radian_);
    odom_.twist.twist.linear.y = CoM_velocity_ *
            sin(heading_yaw_ + CoM_angle_radian_);
}

void OdomController::updateAngularTwist(){
    odom_.twist.twist.angular.z = (velocity_.vector.x /
            CoM_to_rear_) * tan(steer_angle_radian_);
}
