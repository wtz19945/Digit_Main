/*
MPC block
Input : Digit state   (current CoM info)
Output: Digit command (target CoM and Foot Position)
*/

#pragma once
// Eigen pack
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <Eigen/Core>

// ROS pack
#include "ros/ros.h"
#include <ros/package.h>
#include <std_msgs/Int8.h>
#include "Digit_Ros/digit_state.h"
#include "Digit_Ros/mpc_info.h"
#include "input_listener.hpp"


class Digit_MPC {

public:
  Digit_MPC();
  Eigen::VectorXd get_pel_pos() {return pel_pos;};
  Eigen::VectorXd get_pel_vel() {return pel_vel;};
  Eigen::VectorXd get_theta()   {return theta;};
  Eigen::VectorXd get_dtheta()  {return dtheta;};

private:
  void MPCInputCallback(const Digit_Ros::digit_state& msg);

  ros::NodeHandle node_handler;
  ros::Subscriber mpc_sub;

  Eigen::VectorXd pel_pos;
  Eigen::VectorXd pel_vel;
  Eigen::VectorXd theta;
  Eigen::VectorXd dtheta;
};