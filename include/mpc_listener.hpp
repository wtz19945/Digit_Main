/*
MPC Listener block
Input : Digit MPC command
*/

#pragma once
// Eigen pack
#include <Eigen/Dense>
#include <Eigen/Core>

// ROS pack
#include "ros/ros.h"
#include <ros/package.h>
#include <std_msgs/Int8.h>
#include "Digit_Ros/digit_state.h"
#include "Digit_Ros/mpc_info.h"
#include "input_listener.hpp"


class MPC_CMD_Listener {

public:
  MPC_CMD_Listener();
  Eigen::VectorXd get_pel_pos_cmd() {return pel_pos_cmd;};
  Eigen::VectorXd get_pel_vel_cmd() {return pel_vel_cmd;};
  Eigen::VectorXd get_left_foot_cmd()   {return left_foot_cmd;};
  Eigen::VectorXd get_right_foot_cmd()  {return right_foot_cmd;};
  Eigen::VectorXd get_swing_foot_cmd()  {return swing_foot_cmd;};
  double get_dx_offset() {return dx_offset;};

private:
  void MPCInputCallback(const Digit_Ros::mpc_info& msg);

  ros::NodeHandle node_handler;
  ros::Subscriber mpc_cmd_sub;

  Eigen::VectorXd pel_pos_cmd;
  Eigen::VectorXd pel_vel_cmd;
  Eigen::VectorXd left_foot_cmd;
  Eigen::VectorXd right_foot_cmd;
  Eigen::VectorXd swing_foot_cmd;
  double dx_offset;
};