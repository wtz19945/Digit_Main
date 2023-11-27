#pragma once

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int32.h"

class InputListener {


public:
  InputListener(int* key_mode);
  int* key_mode; 
  //int keyvalue = 0;
private:
  void keyInputCallback(const std_msgs::Int32::ConstPtr& msg);

  ros::NodeHandle node_handler;
  ros::Subscriber key_sub;

};