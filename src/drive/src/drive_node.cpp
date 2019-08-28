#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Joy.h>

static const float max_steer_angle = 1900;
static const float center_steer_angle = 1500;
static const float min_steer_angle = 1100;

static const float max_drive_speed = 1600;
static const float center_drive_speed = 1500;
static const float min_drive_speed = 1000;

static const float max_trigger_angle    =  2.0;
static const float center_trigger_angle =  0.0;
static const float min_trigger_angle    = -2.0;

static const float max_joy_angle    =  1.0;
static const float center_joy_angle =  0.0;
static const float min_joy_angle    = -1.0;

static const int right_trigger_axis = 4;
static const int left_trigger_axis = 5;
static const int steer_axis = 0;


class DriveController {
public:
  DriveController();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  ros::NodeHandle nh_;

  ros::Publisher steer_pub_;
  ros::Publisher drive_pub_;
  ros::Subscriber joy_sub_;

};

DriveController::DriveController(){
  steer_pub_ = nh_.advertise<std_msgs::UInt16>("drive/steer", 1);
  drive_pub_ = nh_.advertise<std_msgs::UInt16>("drive/drive", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(
        "joy", 10, &DriveController::joyCallback, this);
}

void DriveController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  std_msgs::UInt16 steer;
  std_msgs::UInt16 drive;

  steer.data = static_cast<uint16_t>(min_steer_angle + 
        ((max_steer_angle - min_steer_angle) 
        / (max_joy_angle - min_joy_angle)) 
        * ((-1.0 * joy->axes[steer_axis]) - min_joy_angle));

  //adjust trigger so left is negative values and right is positive
  float left_trigger_adjusted = joy->axes[left_trigger_axis] - 1;
  float right_trigger_adjusted = 2 - (joy->axes[right_trigger_axis] + 1);
  float combo_trigger = right_trigger_adjusted + left_trigger_adjusted;

  if(combo_trigger > center_trigger_angle){
    drive.data = static_cast<uint16_t>(center_drive_speed + 
        ((max_drive_speed - center_drive_speed) 
        / (max_trigger_angle - center_trigger_angle)) 
        * (combo_trigger - center_trigger_angle));
  
  }else{
    drive.data = static_cast<uint16_t>(min_drive_speed + 
        ((center_drive_speed - min_drive_speed) 
        / (center_trigger_angle - min_trigger_angle)) 
        * (combo_trigger - min_trigger_angle));
  }


  steer_pub_.publish(steer);
  drive_pub_.publish(drive);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "drive_node");
  DriveController drive;

  ros::spin();
}
