#ifndef MPC_MAINV2_H
#define MPC_MAINV2_H
/*
MPC block
Input : Digit state   (current CoM info)
Output: Digit command (target CoM and Foot Position)
*/

#pragma once
// C++
#include <iostream>
#include <vector>
#include <chrono>

// Eigen pack
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <filesystem>

// ROS pack
#include "ros/ros.h"
#include <ros/package.h>
#include <std_msgs/Int8.h>
#include "Digit_Ros/digit_state.h"
#include "Digit_Ros/mpc_info.h"
#include "input_listener.hpp"
#include "cpptoml/include/cpptoml.h"

// casadi pack
#include <casadi/casadi.hpp>

// custom pack
#include "mpc_solver.hpp"

class Digit_MPC {

public:
  Digit_MPC();
  Eigen::VectorXd get_pel_pos() {return pel_pos_;};
  Eigen::VectorXd get_pel_vel() {return pel_vel_;};
  Eigen::VectorXd get_theta()   {return theta_;};
  Eigen::VectorXd get_dtheta()  {return dtheta_;};
  Eigen::VectorXd get_pel_ref()  {return pel_ref_;};
  Eigen::VectorXd get_foot_pos()  {return foot_pos_;};
  Eigen::VectorXd get_obs_info()  {return obs_info_;};
  double get_traj_time()  {return traj_time_;};
  double get_foot_wdith() {return f_width_;};
  double get_steptime() {return step_time_;};
  double get_dstime() {return ds_time_;};
  int get_stance_leg() {return stance_leg_;};
  int get_Var_Num() {return Vars_Num_;};
  Eigen::VectorXd Update_MPC_(int traj_time, std::vector<std::vector<double>> mpc_input);

private:
  void MPCInputCallback(const Digit_Ros::digit_state& msg);

  ros::NodeHandle node_handler_;
  ros::Subscriber mpc_sub_;

  // MPC input info
  Eigen::VectorXd pel_pos_;
  Eigen::VectorXd pel_vel_;
  Eigen::VectorXd theta_;
  Eigen::VectorXd dtheta_;
  Eigen::VectorXd pel_ref_;
  Eigen::VectorXd foot_pos_;
  Eigen::VectorXd obs_info_;
  double traj_time_;
  int stance_leg_;

  casadi::Function left_step0_matrix_; 
  casadi::Function left_step1_matrix_;
  casadi::Function left_step2_matrix_;
  casadi::Function left_step3_matrix_;

  casadi::Function right_step0_matrix_;
  casadi::Function right_step1_matrix_;
  casadi::Function right_step2_matrix_;
  casadi::Function right_step3_matrix_;

  // Conversion between casadi DM and Eigen is not available. Use cpp vector instead
  // QP Parameters
  std::vector<double> f_length_;   // stride max length
  std::vector<double> Weights_ss_; // single support 
  std::vector<double> Weights_ds_; // double support
  std::vector<double> r_;          // obstacle radius
  double f_width_;
  
  // MPC
  int Nodes_;
  int nx_;
  int NPred_;
  double ds_time_;
  double step_time_;
  // OSQP Solver
  MPC_Solver mpc_solver0_;
  MPC_Solver mpc_solver1_;
  MPC_Solver mpc_solver2_;
  MPC_Solver mpc_solver3_;

  std::vector<int> Cons_Num_;
  int Vars_Num_;
  // QP solution
  std::vector<double> sol_;
};
#endif //MPC_MAINV2_H