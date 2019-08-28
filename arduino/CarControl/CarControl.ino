/*
 * MAKE SURE TO USE THE OLD BOOTLOADER!!
 * Written by Jason Ashton, Sean Hunt, Myles Spencer
 */
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "RunningMedian.h"
#include <Servo.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

static const float timeout_ms = 250;

static const int led_pin = 13;
static const int encoder_a_pin = 2;
static const int encoder_b_pin = 3;
static const int steer_servo_pin = 9;
static const int drive_servo_pin = 10;

static const float inner_diameter_mm = 65.8;
static const float outer_diameter_mm = 108.7;
static const float inner_magnet_arclen_mm = 6.35;
static const float inner_non_magnet_arclen_mm = 19.37;
static const float outer_magnet_arclen_m = ((outer_diameter_mm / inner_diameter_mm) * inner_magnet_arclen_mm) / 1000;
static const float outer_non_magnet_arclen_m = ((outer_diameter_mm / inner_diameter_mm) * inner_non_magnet_arclen_mm) / 1000;

/*
 * Dynamic Variables
 */
 
Servo steer;
Servo drive;

ros::NodeHandle nh;
geometry_msgs::Vector3Stamped velocity_msg;

int vel_dir = 1;
double delta_t = 0;
float velocity;
volatile byte a_state;
volatile byte b_state;
unsigned long prev_time;

RunningMedian vel_samples = RunningMedian(10);

/*
 * Callbacks & Sub/Pub
 */

void steer_cb( const std_msgs::UInt16& cmd_msg){
  steer.writeMicroseconds(cmd_msg.data);
}

void drive_cb( const std_msgs::UInt16& cmd_msg){
  drive.writeMicroseconds(cmd_msg.data);
}

ros::Publisher velocity_pub("encoder/velocity", &velocity_msg);
ros::Subscriber<std_msgs::UInt16> steer_sub("drive/steer", steer_cb);
ros::Subscriber<std_msgs::UInt16> drive_sub("drive/drive", drive_cb);

/*
 * Helper Functions
 */

void calculate_velocity(byte new_a_state){
  if(new_a_state == a_state){ return; }
  const double current_time = millis();
  delta_t = current_time - prev_time;
  const double delta_t_sec = delta_t / 1000;

  float new_velocity = 0;
  if(new_a_state == HIGH){ // from on magnet to off
    new_velocity = outer_magnet_arclen_m / delta_t_sec;
  }
  else if(new_a_state == LOW){ // from off magnet to on
    new_velocity = (outer_non_magnet_arclen_m) / delta_t_sec;
  }
  vel_samples.add(vel_dir * new_velocity);
  velocity = new_velocity;
  prev_time = current_time;
  a_state = new_a_state;
}

void interruptA(){
  byte new_a_state = digitalRead(encoder_a_pin);
  digitalWrite(led_pin, new_a_state);
  calculate_velocity(new_a_state);
}

void interruptB(){
  b_state = digitalRead(encoder_b_pin);
  if(b_state && a_state){
    vel_dir = -1;
  }else{
    vel_dir = 1;
  }
}

void setup(){
  nh.getHardware()->setBaud(38400);
  nh.initNode();
  nh.subscribe(steer_sub);
  nh.subscribe(drive_sub);

  nh.advertise(velocity_pub);

  steer.attach(steer_servo_pin); //attach it to pin 9
  drive.attach(drive_servo_pin);

  pinMode(led_pin, OUTPUT);
  pinMode(encoder_a_pin, INPUT_PULLUP);
  pinMode(encoder_b_pin, INPUT_PULLUP);

  a_state = digitalRead(encoder_a_pin);
  b_state = digitalRead(encoder_b_pin);

  prev_time = millis();

  attachInterrupt(digitalPinToInterrupt(encoder_a_pin), interruptA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_b_pin), interruptB, CHANGE);
}

void loop(){
  //velocity_msg.data = velocity;
  velocity_msg.vector.x = vel_samples.getMedian();
  velocity_msg.header.stamp = nh.now();
  velocity_pub.publish(&velocity_msg);

  if(millis() - prev_time > (delta_t + timeout_ms)){ vel_samples.add(0); }

  nh.spinOnce();
  delay(1);
}
