/*
 * Modified OSC standing controller. Changed OSC structure and assumed fixed indexing for matrix non-zeros for faster computation.
 */


#include <stdio.h>
#include <chrono>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <future>
#include <ctime>
#include <sstream>

// OSQP and Eigen
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <Eigen/Core>

// Custom Code
#include "../lowlevelapi.h"
#include "analytical_expressions.hpp"
#include "kin_left_arm.hpp"
#include "kin_right_arm.hpp"
#include "cpptoml/include/cpptoml.h"
#include "Digit_safety.hpp"
#include "OSC_Control.hpp"
#include "OSC_ControlV2.hpp"
#include "Filter.hpp"
#include "mpc_listener.hpp"

// ros part
#include "ros/ros.h"
#include <ros/package.h>
#include <std_msgs/Int8.h>
#include "Digit_Ros/digit_state.h"
#include "input_listener.hpp"
#include "rosbag/bag.h"
//#include "toml.hpp"

// TODO: Move to head file
namespace joint{
  int left_hip_roll = 0;
  int left_hip_yaw = 1;
  int left_hip_pitch = 2;
  int left_knee = 3;
  int left_toe_A = 4;
  int left_toe_B = 5;
  int right_hip_roll = 6;
  int right_hip_yaw = 7;
  int right_hip_pitch = 8;
  int right_knee = 9;
  int right_toe_A = 10;
  int right_toe_B = 11;
  int left_shoulder_roll = 12;
  int left_shoulder_pitch = 13;
  int left_shoulder_yaw = 14;
  int left_elbow = 15;
  int right_shoulder_roll = 16;
  int right_shoulder_pitch = 17;
  int right_shoulder_yaw = 18;
  int right_elbow = 19;

  int left_shin = 0;
  int left_tarsus = 1;
  int left_toe_pitch = 2;
  int left_toe_roll = 3;
  int right_shin = 5;
  int right_tarsus = 6;
  int right_toe_pitch = 7;
  int right_toe_roll = 8;
}

namespace wbc{
  int pel_x = 0;
  int pel_y = 1;
  int pel_z = 2;
  int pel_rotz = 3;
  int pel_roty = 4;
  int pel_rotx = 5;

  int left_hip_roll = 6;
  int left_hip_yaw = 7;
  int left_hip_pitch = 8;
  int left_knee = 9;
  int left_tarsus = 10;
  int left_toe_pitch = 11;
  int left_toe_roll = 12;

  int right_hip_roll = 13;
  int right_hip_yaw = 14;
  int right_hip_pitch = 15;
  int right_knee = 16;
  int right_tarsus = 17;
  int right_toe_pitch = 18;
  int right_toe_roll = 19;
}

static double target_position[] = {
  -0.0462933,
  -0.0265814,
  0.19299,
  -0.3,
  -0.0235182,
  -0.0571617,//left foot
  -0.0125,
  0.18,
  0.33,
  0.83,
  -0.485182,
  0.2371617,//right foot#include "analytical_expressions.hpp"

  -0.3,
  0.943845,
  0.0,
  0.3633,//left arm
  0.3,
  -0.943845,
  0.0,
  -0.3633,//right arm
};

MatrixXd get_B(VectorXd q);
MatrixXd get_Spring_Jaco();
VectorXd ToEulerAngle(VectorXd q);
MatrixXd get_fric_constraint(double mu);
MatrixXd get_pr2m_jaco(VectorXd a, VectorXd b, double x, double y);
VectorXd get_quintic_params(VectorXd x0, VectorXd xT, double T);


#define NUM_FROST_STATE 28
#define NUM_Dyn_STATE 20
#define NUM_LEG_STATE 14
#define NUM_PEL_STATE 6
#define NUM_ARM_STATE 8

using namespace std;
using namespace std::chrono;
using namespace Eigen;

int main(int argc, char* argv[])
{
  

  OsqpEigen::Solver solver;
  int QP_initialized = 0;

  //joint readings
  AnalyticalExpressions analytical_expressions;
  VectorXd q = VectorXd::Zero(NUM_MOTORS,1);
  VectorXd dq = VectorXd::Zero(NUM_MOTORS,1);
  VectorXd qj = VectorXd::Zero(NUM_JOINTS,1);
  VectorXd dqj = VectorXd::Zero(NUM_JOINTS,1);
  VectorXd torq = VectorXd::Zero(NUM_MOTORS,1);

  // joint used in FROST is 28, 3 base position, 3 base orientation, 22 (6 floating base, 12 actuated joints, 4 passive joints). Assuming fixed arm
  VectorXd wb_q = VectorXd::Zero(NUM_FROST_STATE,1);
  VectorXd wb_dq = VectorXd::Zero(NUM_FROST_STATE,1);
  VectorXd pb_q = VectorXd::Zero(NUM_Dyn_STATE,1);
  VectorXd pb_dq = VectorXd::Zero(NUM_Dyn_STATE,1);

  // Position estimator reading from LLAPI
  VectorXd pel_pos(3),pel_vel(3),pel_quaternion(4),theta(3),dtheta(3);
  pel_pos = VectorXd::Zero(3,1);
  pel_vel = VectorXd::Zero(3,1);
  pel_quaternion = VectorXd::Zero(4,1);
  theta = VectorXd::Zero(3,1);
  dtheta = VectorXd::Zero(3,1);
  
  // initialize ros
  ros::init(argc, argv, "sample_node");
  ros::NodeHandle n;
  bool run_sim = true;
  n.getParam("sim_mode",run_sim);

  // The publisher address should be changed to the ip address of the robot
  const char* publisher_address = "";  
  if (run_sim){
    publisher_address = "127.0.0.1";   // simulator address
  }else
  { 
    publisher_address = "10.10.1.1";   // hardware address
  }
  llapi_init(publisher_address);

  // Define inputs and outputs (updated each iteration)
  llapi_command_t command = {0};
  llapi_observation_t observation;

  // Connect to robot (need to send commands until the subscriber connects)
  command.apply_command = false;
  while (!llapi_get_observation(&observation)) llapi_send_command(&command);

  // Get local copy of command limits (torque and damping)
  const llapi_limits_t* limits = llapi_get_limits();
  
  // Load Controller Gains from TOML file
  std::string package_path; 
  try {
    package_path = ros::package::getPath("Digit_Ros");
    if (package_path.empty()) {
      throw 1;
    }
  } catch(...) {
    std::cerr << "package not found\n";
    return 0;
  }
  std::shared_ptr<cpptoml::table> config = cpptoml::parse_file(package_path + "/src/config_file/oscmpc_robot_config.toml");

  // OSC base gain
  double cpx = config->get_qualified_as<double>("PD-Gains.com_P_gain_x").value_or(0);
  double cpy = config->get_qualified_as<double>("PD-Gains.com_P_gain_y").value_or(0);
  double cpz = config->get_qualified_as<double>("PD-Gains.com_P_gain_z").value_or(0);
  double cprz = config->get_qualified_as<double>("PD-Gains.com_P_gain_rz").value_or(0);
  double cpry = config->get_qualified_as<double>("PD-Gains.com_P_gain_ry").value_or(0);
  double cprx = config->get_qualified_as<double>("PD-Gains.com_P_gain_rx").value_or(0);
  double cphy = config->get_qualified_as<double>("PD-Gains.hip_yaw_P").value_or(0);

  double cdx = config->get_qualified_as<double>("PD-Gains.com_D_gain_x").value_or(0);
  double cdy = config->get_qualified_as<double>("PD-Gains.com_D_gain_y").value_or(0);
  double cdz = config->get_qualified_as<double>("PD-Gains.com_D_gain_z").value_or(0);
  double cdrz = config->get_qualified_as<double>("PD-Gains.com_D_gain_rz").value_or(0);
  double cdry = config->get_qualified_as<double>("PD-Gains.com_D_gain_ry").value_or(0);
  double cdrx = config->get_qualified_as<double>("PD-Gains.com_D_gain_rx").value_or(0);
  double cdhy = config->get_qualified_as<double>("PD-Gains.hip_yaw_D").value_or(0);

  double fcdx = config->get_qualified_as<double>("PD-Gains.crt_com_D_gain_x").value_or(0);
  double fcdy = config->get_qualified_as<double>("PD-Gains.crt_com_D_gain_y").value_or(0);
  // IK arm gain
  double arm_P = config->get_qualified_as<double>("PD-Gains.arm_P").value_or(0);

  // Foot Params
  double cp_py  = config->get_qualified_as<double>("PD-Gains.cp_P").value_or(0);
  double mpc_px = config->get_qualified_as<double>("PD-Gains.mpc_px").value_or(0);
  double mpc_py = config->get_qualified_as<double>("PD-Gains.mpc_py").value_or(0);

  double fpx = config->get_qualified_as<double>("PD-Gains.foot_P_x").value_or(0);
  double fpy = config->get_qualified_as<double>("PD-Gains.foot_P_y").value_or(0);
  double fpz = config->get_qualified_as<double>("PD-Gains.foot_P_z").value_or(0);
  double fdx = config->get_qualified_as<double>("PD-Gains.foot_D_x").value_or(0);
  double fdy = config->get_qualified_as<double>("PD-Gains.foot_D_y").value_or(0);
  double fdz = config->get_qualified_as<double>("PD-Gains.foot_D_z").value_or(0);

  double Wcom = config->get_qualified_as<double>("QP-Params.com_W").value_or(0);
  double Wff  = config->get_qualified_as<double>("QP-Params.st_foot_W").value_or(0);
  double Wfb  = config->get_qualified_as<double>("QP-Params.st_foot_W").value_or(0);

  double force_max = config->get_qualified_as<double>("QP-Params.force_max").value_or(0);
  double mu = config->get_qualified_as<double>("QP-Params.mu").value_or(0);

  int wd_sz = config->get_qualified_as<double>("Filter.wd_sz").value_or(0);
  double init_count = config->get_qualified_as<double>("Start.init_count").value_or(0);
  double soft_count = config->get_qualified_as<double>("Start.soft_count").value_or(0);

  // Arm params
  double arm_z_int = config->get_qualified_as<double>("Arm-IK.z_int").value_or(0);
  double arm_z_mag = config->get_qualified_as<double>("Arm-IK.z_mag").value_or(0);
  double arm_z_prd = config->get_qualified_as<double>("Arm-IK.z_prd").value_or(0);

  // swing foot profile
  double step_time = config->get_qualified_as<double>("Walk-Params.step_time").value_or(0);
  double zh = config->get_qualified_as<double>("Walk-Params.step_height").value_or(0);
  double zend = config->get_qualified_as<double>("Walk-Params.end_pos").value_or(0);
  double dzend = config->get_qualified_as<double>("Walk-Params.end_vel").value_or(0);
  double ddzend = config->get_qualified_as<double>("Walk-Params.end_acc").value_or(0);
  double ds_time = config->get_qualified_as<double>("Walk-Params.ds_time").value_or(0);

  // osc qp rate and version
  double qp_rate = config->get_qualified_as<double>("QP-Params.qp_rate").value_or(0);
  int osc_version = config->get_qualified_as<double>("QP-Params.osc_version").value_or(0);
  int recording = config->get_qualified_as<double>("Other.recording").value_or(0);

  // Weight Matrix and Gain Vector
  MatrixXd Weight_ToeF = Wff*MatrixXd::Identity(6,6);
  VectorXd KP_ToeF = VectorXd::Zero(6,1);
  VectorXd KD_ToeF = VectorXd::Zero(6,1);
  KP_ToeF << fpx,fpy,fpz,fpx,fpy,fpz;
  KD_ToeF << fdx,fdy,fdz,fdx,fdy,fdz;

  MatrixXd Weight_ToeB = Wfb*MatrixXd::Identity(8,8);
  VectorXd KP_ToeB = KP_ToeF;
  VectorXd KD_ToeB = KD_ToeF;

  MatrixXd Weight_pel = Wcom * MatrixXd::Identity(6,6);
  VectorXd KP_pel = VectorXd::Zero(6,1);
  VectorXd KD_pel = VectorXd::Zero(6,1);
  KP_pel << cpx,cpy,cpz,cprz,cpry,cprx;
  KD_pel << cdx,cdy,cdz,cdrz,cdry,cdrx;

  // Incorporate damping command into OSC
  double damping_dt = 1 / qp_rate;
  VectorXd damping = VectorXd::Zero(20,1);
  VectorXd D_term = VectorXd::Zero(20,1);
  damping << VectorXd::Zero(6,1), 66.849, 26.1129, 38.05, 38.05, 0 , 0, 0, 
              66.849, 26.1129, 38.05, 38.05, 0 , 0, 0;
  MatrixXd Dmat = damping.asDiagonal();

  // initialize Digit foot and base states
  double yaw_des = 0;
  double pel_x = 0;
  double pel_y = 0;
  double pel_z = 0;
  double vel_x = 0;
  double vel_y = 0;
  double vel_z = 0;
  double z_off = 0;
  double z_off_track = 0;

  VectorXd left_toe_pos = VectorXd::Zero(3,1);
  VectorXd left_toe_back_pos = VectorXd::Zero(3,1);
  VectorXd right_toe_pos = VectorXd::Zero(3,1);
  VectorXd right_toe_back_pos = VectorXd::Zero(3,1);

  MatrixXd M = analytical_expressions.InertiaMatrix(pb_q);
  MatrixXd G = analytical_expressions.GravityVector(pb_q);
  // toe front kinematics
  MatrixXd left_toe_jaco  = analytical_expressions.Jp_left_toe_front(wb_q);
  MatrixXd right_toe_jaco = analytical_expressions.Jp_right_toe_front(wb_q);
  // toe back kinematics
  MatrixXd left_toe_back_jaco  = analytical_expressions.Jp_left_toe_back(wb_q);
  MatrixXd right_toe_back_jaco  = analytical_expressions.Jp_right_toe_back(wb_q);

  int update_mat = -1;

  // initialize safety checker
  Digit_safety safe_check(wd_sz,NUM_LEG_STATE);

  // initialize Filter
  MovingAverageFilter pel_vel_x; 
  MovingAverageFilter pel_vel_y;
  vector<MovingAverageFilter> wb_dq_fil;
  for(int i = 0;i < NUM_LEG_STATE;i++){
    wb_dq_fil.push_back(MovingAverageFilter(5));
  }
  vector<MovingAverageFilter> f_vel_fil;
  for(int i=0;i<12;i++){
    f_vel_fil.push_back(MovingAverageFilter(5));
  }

  // Set ROS params
  ros::Rate control_loop_rate(qp_rate);
  ros::Publisher state_pub = n.advertise<Digit_Ros::digit_state>("digit_state", 10);
  rosbag::Bag bag;

  std::time_t now = std::time(nullptr);
  std::stringstream date;
  date << std::put_time(std::localtime(&now), "%Y_%m_%d_%H_%M_%S");
  if(recording){
    if(run_sim)
      bag.open(package_path + "/data/Sim_Data/sim_test_" + date.str() + ".bag", rosbag::bagmode::Write);
    else
      bag.open(package_path + "/data/Hard_Data/test_" + date.str() + ".bag", rosbag::bagmode::Write);
  }
   
  // keyboard input time tracker
  double key_time_tracker = 0; // time log helper for key input, might remove in the future
  int key_mode = -1;
  int key_mode_prev = -1;
  InputListener input_listener(&key_mode);
  MPC_CMD_Listener mpc_cmd_listener;

  // arm IK tracker
  VectorXd ql_last = VectorXd::Zero(10,1);
  VectorXd qr_last = VectorXd::Zero(10,1);

  // OSC and Walking Variables
  OSC_Control osc(config);
  OSC_ControlV2 oscV2(config);

  VectorXd QPSolution = VectorXd::Zero(20,1);
  VectorXd last_QPSolution = VectorXd::Zero(20,1);

  VectorXd pel_pos_des = VectorXd::Zero(3,1);
  VectorXd pel_vel_des = VectorXd::Zero(3,1);
  VectorXd pel_omg_des = VectorXd::Zero(3,1);

  VectorXd fzint(3) ; fzint << 0,0,0;
  VectorXd fzmid(3) ; fzmid << zh,0,0;
  VectorXd fzend(3) ; fzend << 0,dzend,ddzend;
  VectorXd a = get_quintic_params(fzint,fzmid,step_time/2 - ds_time/2);
  VectorXd b = get_quintic_params(fzmid,fzend,step_time/2 - ds_time/2);
  VectorXd foot_start = VectorXd::Zero(2,1);
  VectorXd ya = VectorXd::Zero(6,1);
  VectorXd xa = VectorXd::Zero(6,1);
  VectorXd tvec = VectorXd::Zero(6,1);
  VectorXd dtvec = VectorXd::Zero(6,1);
  VectorXd ddtvec = VectorXd::Zero(6,1);
  VectorXd contact = VectorXd::Zero(2,1);
  contact << 1,1;
  int stance_leg = 1;
  int stepping = 0;
  int change_state = 0;
  double traj_time = 0;
  VectorXd pos_avg = VectorXd::Zero(3,1);
  // computation time tracker
  auto time_control_start = std::chrono::system_clock::now();
  double digit_time_start = observation.time;
  double digit_time_last = observation.time;
  double circle_time = 0;

  while (ros::ok()) {
    // count running time
    auto elapsed_time = duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - time_control_start);
    auto time_program_start = std::chrono::system_clock::now();
    double global_time = elapsed_time.count() / 1e6;

    // Update observation
    int return_val = llapi_get_observation(&observation);
    if (return_val < 1) {
      // Error occurred
    } else if (return_val) {
      // New data received
    } else {
      // No new data
    }

    circle_time = (observation.time - digit_time_last);
    // get contact trajectory
    if(key_mode == 4 || stepping > 0){
      traj_time = traj_time + circle_time; // use this when the simulator time is slow than real-time
      //traj_time = traj_time + 1 / qp_rate; // use this when the simualator is close to real-time
      if(traj_time > step_time){
        traj_time = 0;
        change_state = 1;
      }
      else{
        change_state = 0;
      }
      if(stepping<2 && change_state == 1)
        stepping++;
    }

    if(stepping == 0){
      contact << 1,1;
    }
    else if(stepping == 1){
      contact << 1,1;
    }
    else{
      if(change_state){
        stance_leg *= -1;
      }

      if(stance_leg == 1){
        if(ds_time == 0){
            contact << 0,1;
        }
        else{
          if(traj_time <= ds_time/2)
            contact << 0.5 - traj_time * 1 / ds_time, 0.5 + traj_time * 1 / ds_time;
          else if(traj_time >= step_time - ds_time/2)
            contact << 0 + 1 / ds_time * (traj_time - step_time + ds_time/2),1 - 1 / ds_time * (traj_time - step_time + ds_time/2);
          else
            contact << 0,1;
        }
      }
      else{
        if(ds_time == 0){
            contact << 1,0;
        }
        else{
          if(traj_time <= ds_time/2)
            contact << 0.5 + traj_time * 1 / ds_time, 0.5 - traj_time * 1 / ds_time;
          else if(traj_time >= step_time - ds_time/2)
            contact << 1 - 1 / ds_time * (traj_time - step_time + ds_time/2), 0 + 1 / ds_time * (traj_time - step_time + ds_time/2);
          else
            contact << 1,0;
        }
      }
    }
    digit_time_last = observation.time;

    // Get state information
    for (int i = 0; i < NUM_MOTORS; i++){
      q(i) = observation.motor.position[i];
      dq(i) = observation.motor.velocity[i];
      torq(i) = observation.motor.torque[i];
    }

    for (int i = 0; i < NUM_JOINTS; i++){
      qj(i) = observation.joint.position[i];
      dqj(i) = observation.joint.velocity[i];
    }
    
    for (int i = 0;i < 3;i++){
        pel_pos(i) = observation.base.translation[i];
        pel_vel(i) = observation.base.linear_velocity[i];
        dtheta(i) = observation.base.angular_velocity[i];
    }
    pel_quaternion(0) = observation.base.orientation.w;
    pel_quaternion(1) = observation.base.orientation.x;
    pel_quaternion(2) = observation.base.orientation.y;
    pel_quaternion(3) = observation.base.orientation.z;
    
    // Cmd
    if(key_mode == 2){
      yaw_des += 0.2/qp_rate;
      pel_omg_des(2) = 0.2/qp_rate;
    }
      
    if(key_mode == 3){
      yaw_des -= 0.2/qp_rate;
      pel_omg_des(2) = 0.2/qp_rate;
    }

    // Wrap theta
    if(yaw_des> M_PI){
        yaw_des -= 2 * M_PI; 
    }
    else if(yaw_des < -M_PI){
        yaw_des += 2 * M_PI;
    }
    else{
      //;
    }

    // Frame transformation
    theta = ToEulerAngle(pel_quaternion); // transform quaternion in euler roll, pitch, yaw order
    MatrixXd OmegaToDtheta = MatrixXd::Zero(3,3);
/*     OmegaToDtheta << 0 , -sin(theta(2)), cos(theta(2)) * cos(theta(1)), 0, cos(theta(2)), cos(theta(1)) * sin(theta(2)), 1, 0, -sin(theta(1));
    dtheta = OmegaToDtheta * dtheta; */
    OmegaToDtheta << cos(theta(2)) * cos(theta(1)), -sin(theta(2)), 0, sin(theta(2)) *cos(theta(1)), cos(theta(2)), 0, -sin(theta(1)), 0, 1;
    dtheta = OmegaToDtheta.transpose() * dtheta;
    MatrixXd rotZ = MatrixXd::Zero(3,3);
    rotZ << cos(yaw_des),-sin(yaw_des),0,sin(yaw_des),cos(yaw_des),0,0,0,1;

    //pel_vel(0) = pel_vel_x.getData(pel_vel(0));
    //pel_vel(1) = pel_vel_y.getData(pel_vel(1));

    // Wrap yaw orientation and positions so the desired states are always 0
    if((observation.time - digit_time_start) < init_count){
        yaw_des = theta(2);
        pel_x = pel_pos(0);
        pel_y = pel_pos(1);
        pel_z = pel_pos(2);
    }

    pel_pos(0) -= pel_x;
    pel_pos(1) -= pel_y;
    theta(2) -= yaw_des;

    // pelvis states in the base frame
    pel_pos = rotZ.transpose() * pel_pos; 
    pel_vel = rotZ.transpose() * pel_vel; // hardware: use body or world frame??? Both seems working

    // Wrap theta
    if(theta(2) > M_PI){
        theta(2) -= 2 * M_PI; 
    }
    else if(theta(2) < -M_PI){
        theta(2) += 2 * M_PI;
    }
    else{
      //;
    }

/*     if((observation.time - digit_time_start) >= init_count){
      pel_pos(0) -= pos_avg(0); 
      pel_pos(1) -= pos_avg(1);
    }
     */
    // get state vector
    wb_q  << pel_pos, theta(2), theta(1), theta(0), q(joint::left_hip_roll),q(joint::left_hip_yaw),q(joint::left_hip_pitch),q(joint::left_knee)
      ,qj(joint::left_tarsus),qj(joint::left_toe_pitch),qj(joint::left_toe_roll),
      q(joint::right_hip_roll),q(joint::right_hip_yaw),q(joint::right_hip_pitch),q(joint::right_knee),
      qj(joint::right_tarsus),qj(joint::right_toe_pitch),qj(joint::right_toe_roll),q(joint::left_shoulder_roll),q(joint::left_shoulder_pitch)
      ,q(joint::left_shoulder_yaw),q(joint::left_elbow),q(joint::right_shoulder_roll),q(joint::right_shoulder_pitch)
      ,q(joint::right_shoulder_yaw),q(joint::right_elbow);

    wb_dq  << pel_vel, dtheta(2), dtheta(1), dtheta(0), dq(joint::left_hip_roll),dq(joint::left_hip_yaw),dq(joint::left_hip_pitch),dq(joint::left_knee)
      ,dqj(joint::left_tarsus),dqj(joint::left_toe_pitch),dqj(joint::left_toe_roll),
      dq(joint::right_hip_roll),dq(joint::right_hip_yaw),dq(joint::right_hip_pitch),dq(joint::right_knee),
      dqj(joint::right_tarsus),dqj(joint::right_toe_pitch),dqj(joint::right_toe_roll),dq(joint::left_shoulder_roll),dq(joint::left_shoulder_pitch)
      ,dq(joint::left_shoulder_yaw),dq(joint::left_elbow),dq(joint::right_shoulder_roll),dq(joint::right_shoulder_pitch)
      ,dq(joint::right_shoulder_yaw),dq(joint::right_elbow);
    

    pb_q << pel_pos, theta(2), theta(1), theta(0), q(joint::left_hip_roll),q(joint::left_hip_yaw),q(joint::left_hip_pitch),q(joint::left_knee)
      ,qj(joint::left_tarsus),qj(joint::left_toe_pitch),qj(joint::left_toe_roll),q(joint::right_hip_roll),q(joint::right_hip_yaw),q(joint::right_hip_pitch)
      ,q(joint::right_knee),qj(joint::right_tarsus),qj(joint::right_toe_pitch),qj(joint::right_toe_roll);

    pb_dq << pel_vel, dtheta(2), dtheta(1), dtheta(0), dq(joint::left_hip_roll),dq(joint::left_hip_yaw),dq(joint::left_hip_pitch),dq(joint::left_knee)
      ,dqj(joint::left_tarsus),dqj(joint::left_toe_pitch),dqj(joint::left_toe_roll),dq(joint::right_hip_roll),dq(joint::right_hip_yaw),dq(joint::right_hip_pitch)
      ,dq(joint::right_knee),dqj(joint::right_tarsus),dqj(joint::right_toe_pitch),dqj(joint::right_toe_roll);
      
    // height compensation for drifting assumeing fixed ground height
    if(contact(0) > 0 && contact(1) > 0){
        pel_pos(2) += pos_avg(2) - (left_toe_pos(2) + right_toe_pos(2) + left_toe_back_pos(2) + right_toe_back_pos(2)) / 4;
    }
    if(contact(0) > 0 && contact(1) == 0){
        pel_pos(2) += pos_avg(2) - (left_toe_pos(2) + left_toe_back_pos(2)) / 2;
        //right_toe_pos(2) += pos_avg(2) - (left_toe_pos(2) + left_toe_back_pos(2)) / 2;
        //right_toe_back_pos(2) += pos_avg(2) - (left_toe_pos(2) + left_toe_back_pos(2)) / 2;
    }
    if(contact(0) == 0 && contact(1) > 0){
        pel_pos(2) += pos_avg(2) - (right_toe_pos(2) + right_toe_back_pos(2)) / 2;
        //left_toe_pos(2) += pos_avg(2) - (right_toe_pos(2) + right_toe_back_pos(2)) / 2;
        //left_toe_back_pos(2) += pos_avg(2) - (right_toe_pos(2) + right_toe_back_pos(2)) / 2;
    }
 
    // compute end effector position
    // VectorXd pelvis_pos = analytical_expressions.p_Pelvis(wb_q);
    
    // Compute Dynamics and kinematics, partial update???
    if(update_mat == -1){
      M = analytical_expressions.InertiaMatrix(pb_q);
      G = analytical_expressions.GravityVector(pb_q);

      // toe front kinematics
      left_toe_pos = analytical_expressions.p_left_toe_front(wb_q);
      left_toe_jaco  = analytical_expressions.Jp_left_toe_front(wb_q);
      //MatrixXd left_toe_djaco  = analytical_expressions.dJp_left_toe_front(wb_q,wb_dq);
      right_toe_pos = analytical_expressions.p_right_toe_front(wb_q);    
      right_toe_jaco = analytical_expressions.Jp_right_toe_front(wb_q);
      //MatrixXd right_toe_djaco = analytical_expressions.dJp_right_toe_front(wb_q,wb_dq);

      // toe back kinematics
      left_toe_back_pos = analytical_expressions.p_left_toe_back(wb_q);
      left_toe_back_jaco  = analytical_expressions.Jp_left_toe_back(wb_q);
      //MatrixXd left_toe_back_djaco  = analytical_expressions.dJp_left_toe_back(wb_q,wb_dq);
      right_toe_back_pos = analytical_expressions.p_right_toe_back(wb_q);
      right_toe_back_jaco  = analytical_expressions.Jp_right_toe_back(wb_q);
      //MatrixXd right_toe_back_djaco  = analytical_expressions.dJp_right_toe_back(wb_q,wb_dq); 
    }

    // Get fixed arm version
    MatrixXd left_toe_jaco_fa = MatrixXd::Zero(3,20); // fa: fixed arm
    MatrixXd left_toe_back_jaco_fa = MatrixXd::Zero(3,20);
    MatrixXd right_toe_jaco_fa = MatrixXd::Zero(3,20);
    MatrixXd right_toe_back_jaco_fa = MatrixXd::Zero(3,20);

    left_toe_jaco_fa << left_toe_jaco.block(0,0,3,13) , MatrixXd::Zero(3,7);
    left_toe_back_jaco_fa << left_toe_back_jaco.block(0,0,3,13) , MatrixXd::Zero(3,7);
    right_toe_jaco_fa << right_toe_jaco.block(0,0,3,6) , MatrixXd::Zero(3,7), right_toe_jaco.block(0,13,3,7);
    right_toe_back_jaco_fa << right_toe_back_jaco.block(0,0,3,6) , MatrixXd::Zero(3,7), right_toe_back_jaco.block(0,13,3,7);

    // Might need this to make sure the desired pelvis is in the center of two foot.
    if((observation.time - digit_time_start)  < init_count){
        pos_avg = (left_toe_pos + right_toe_pos + left_toe_back_pos + right_toe_back_pos) / 4;
        fzint << pos_avg(2),0,0;
        fzmid << pos_avg(2) + zh,0,0;
        fzend << pos_avg(2) + zend,dzend,ddzend;
        a = get_quintic_params(fzint,fzmid,step_time/2 - ds_time/2);
        b = get_quintic_params(fzmid,fzend,step_time/2 - ds_time/2);
    }

    
    // End effector velocity
    VectorXd  left_toe_vel = left_toe_jaco * wb_dq;
    VectorXd  right_toe_vel = right_toe_jaco * wb_dq;

    // Control another contact point on toe back to enable foot rotation control
    VectorXd left_toe_rot  = VectorXd::Zero(4,1);
    VectorXd left_toe_drot = VectorXd::Zero(4,1);
    MatrixXd left_toe_rot_jaco_fa = MatrixXd::Zero(4,20);
    VectorXd right_toe_rot  = VectorXd::Zero(4,1);
    VectorXd right_toe_drot = VectorXd::Zero(4,1);
    MatrixXd right_toe_rot_jaco_fa = MatrixXd::Zero(4,20);

    // Currrently, need to compute the yaw,roll rotation of the leg explicitly
    // Seems to be caused by the toe yaw DOF in Mujoco
    left_toe_rot.block(0,0,3,1) = left_toe_back_pos;
    left_toe_drot.block(0,0,3,1) = left_toe_back_jaco * wb_dq;
    left_toe_rot(3) =  q(joint::left_hip_yaw); 
    left_toe_drot(3) = dq(joint::left_hip_yaw);
    left_toe_rot_jaco_fa.block(0,0,3,20) = left_toe_back_jaco_fa;
    left_toe_rot_jaco_fa(3,wbc::left_hip_yaw) = 1;

    right_toe_rot.block(0,0,3,1) = right_toe_back_pos;
    right_toe_drot.block(0,0,3,1) = right_toe_back_jaco * wb_dq;
    right_toe_rot(3) =  q(joint::right_hip_yaw); 
    right_toe_drot(3) = dq(joint::right_hip_yaw);
    right_toe_rot_jaco_fa.block(0,0,3,20) = right_toe_back_jaco_fa;
    right_toe_rot_jaco_fa(3,wbc::right_hip_yaw) = 1;

    // Filter foot velocity
    left_toe_vel(0) = f_vel_fil[0].getData(left_toe_vel(0));
    left_toe_vel(1) = f_vel_fil[1].getData(left_toe_vel(1));
    left_toe_vel(2) = f_vel_fil[2].getData(left_toe_vel(2));
    left_toe_rot(0) = f_vel_fil[3].getData(left_toe_rot(0));
    left_toe_rot(1) = f_vel_fil[4].getData(left_toe_rot(1));
    left_toe_rot(2) = f_vel_fil[5].getData(left_toe_rot(2));

    right_toe_vel(0) = f_vel_fil[6].getData(right_toe_vel(0));
    right_toe_vel(1) = f_vel_fil[7].getData(right_toe_vel(1));
    right_toe_vel(2) = f_vel_fil[8].getData(right_toe_vel(2));
    right_toe_rot(0) = f_vel_fil[9].getData(right_toe_rot(0));
    right_toe_rot(1) = f_vel_fil[10].getData(right_toe_rot(1));
    right_toe_rot(2) = f_vel_fil[11].getData(right_toe_rot(2));
     

    elapsed_time = duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - time_program_start);
    //cout << "time used to compute system dyn and kin is: " << elapsed_time.count() << endl;
    int use_cap = 0;
    // Compute Desired CoM Traj// Testing use
    if(key_mode == 0){
      z_off = min(z_off_track + 0.1 * (observation.time - key_time_tracker),0.3);
    }
    else if(key_mode == 1){
      z_off = max(z_off_track - 0.1 * (observation.time - key_time_tracker),-0.3);
    }
    else{
      key_time_tracker = observation.time;
      z_off_track = z_off;
    }
    pel_pos_des << 0, 0, pel_z + z_off;

    // saturate position command
    if(abs(pel_pos(2) - pel_pos_des(2)) > 0.04){
        pel_pos_des(2) = pel_pos(2) + (pel_pos_des(2) - pel_pos(2)) / abs(pel_pos(2) - pel_pos_des(2)) * 0.04;
    }

    // transit to stepping
    if(stepping == 1){
       pel_pos_des(1) = 0 + .06 * traj_time;
       pel_vel_des(1) = .06 * traj_time;
    }

    // Stepping direction command
    if(stepping == 2 && use_cap == 0){
      VectorXd mpc_cmd_pel_pos = mpc_cmd_listener.get_pel_pos_cmd();
      VectorXd mpc_cmd_pel_vel = mpc_cmd_listener.get_pel_vel_cmd();

      double lam = 0.0 + 1.0 * (traj_time - ds_time/2) / (step_time - ds_time/2);
      pel_pos_des(0) = (1 - lam) * mpc_cmd_pel_pos(0) + lam * mpc_cmd_pel_pos(1) + pel_pos(0);
      pel_pos_des(1) = (1 - lam) * mpc_cmd_pel_pos(2) + lam  * mpc_cmd_pel_pos(3) + pel_pos(1);
      pel_vel_des(0) = (1 - lam) * mpc_cmd_pel_vel(0) + lam  * mpc_cmd_pel_vel(1);
      pel_vel_des(1) = (1 - lam) * mpc_cmd_pel_vel(2) + lam  * mpc_cmd_pel_vel(3);
    }

    double vel_des_x = -0.0;
    double vel_des_y = -0.0;
    if(stepping == 2){
      // reset position command when walking direction is changed
      if(key_mode == 5){
        vel_des_x = 0.2;
      }
      if(key_mode == 6){
        vel_des_x = -0.2;
      }
      if(key_mode == 7){
        vel_des_y = 0.2;
      }
      if(key_mode == 8){
        vel_des_y = -0.2;
      }      
    }
    key_mode_prev = key_mode;

    // Compute Desired Foot Traj
    VectorXd left_toe_pos_ref = VectorXd::Zero(3,1);
    VectorXd left_toe_vel_ref = VectorXd::Zero(3,1);
    VectorXd left_toe_acc_ref = VectorXd::Zero(3,1);

    VectorXd right_toe_pos_ref = VectorXd::Zero(3,1);
    VectorXd right_toe_vel_ref = VectorXd::Zero(3,1);
    VectorXd right_toe_acc_ref = VectorXd::Zero(3,1);

    if(contact(0) == 0){
      // temporally use capture point
      double x_goal = pel_pos(0) - 0.07 + cp_py * sqrt(.9/9.81) * pel_vel(0) + vel_des_x * 0;
      double y_goal = max(pel_pos(1) + 0.1 + cp_py * sqrt(.9/9.81) * pel_vel(1), (right_toe_pos(1) + right_toe_back_pos(1))/2 + 0.03);

      if(use_cap == 0){
        VectorXd foot_cmd = mpc_cmd_listener.get_left_foot_cmd();
        x_goal = foot_cmd(0) + pel_pos(0);
        y_goal = foot_cmd(1) + pel_pos(1);
      }
      
      double w = M_PI / (step_time - ds_time);
      double n = traj_time - ds_time / 2;
      if(traj_time - ds_time/2 < 0.005)
        foot_start << (left_toe_pos(0) + left_toe_back_pos(0))/2, (left_toe_pos(1) + left_toe_back_pos(1))/2;
      
      double dy_goal  = .5 * y_goal * w * (sin(w * n)) + .5 * w * (-sin(w * n)) * foot_start(1);
      double ddy_goal = .5 * y_goal * w * w * (cos(w * n)) + .5 * w * w * (-cos(w * n)) * foot_start(1);
      y_goal   = .5 * y_goal * (1 - cos(min(mpc_py * w * n,M_PI))) + .5 * (1 + cos(min(mpc_py * w * n,M_PI))) * foot_start(1);

      double dx_goal  = .5 * x_goal * w * (sin(w * n)) + .5 * w * (-sin(w * n)) * foot_start(0);
      double ddx_goal = .5 * x_goal * w * w * (cos(w * n)) + .5 * w * w * (-cos(w * n)) * foot_start(0);
      x_goal   = .5 * x_goal * (1 - cos(min(mpc_px * w * n,M_PI))) + .5 * (1 + cos(min(mpc_px * w * n,M_PI))) * foot_start(0);

      left_toe_pos_ref << x_goal,max(y_goal,(right_toe_pos(1) + right_toe_back_pos(1))/2 + 0.05),0;
      left_toe_vel_ref << dx_goal, dy_goal, 0;
      left_toe_acc_ref << ddx_goal,ddy_goal,0;

      if(traj_time < step_time/2 ){
        double n = traj_time - ds_time / 2;
        tvec   << 1,n,pow(n,2),pow(n,3),pow(n,4),pow(n,5);
        dtvec  << 0,1,2*n,3*pow(n,2),4*pow(n,3),5*pow(n,4);
        ddtvec << 0,0,2,6*n,12*pow(n,2),20*pow(n,3);
        
        left_toe_pos_ref(2) = a.dot(tvec) - (pos_avg(2) -  (right_toe_pos(2) + right_toe_back_pos(2))/2) - 0.0; // reset the ground level to current stance foot position
        left_toe_vel_ref(2) = a.dot(dtvec);
        left_toe_acc_ref(2) = a.dot(ddtvec);
      }
      else{
        double n = traj_time - step_time/2;
        tvec   << 1,n,pow(n,2),pow(n,3),pow(n,4),pow(n,5);
        dtvec  << 0,1,2*n,3*pow(n,2),4*pow(n,3),5*pow(n,4);
        ddtvec << 0,0,2,6*n,12*pow(n,2),20*pow(n,3);
        left_toe_pos_ref(2) = b.dot(tvec) - (pos_avg(2) -  (right_toe_pos(2) + right_toe_back_pos(2))/2) - 0.0; // reset the ground level to current stance foot position
        left_toe_vel_ref(2) = b.dot(dtvec);
        left_toe_acc_ref(2) = b.dot(ddtvec);
      }
    }
    else{
      left_toe_pos_ref = (left_toe_pos + left_toe_back_pos) / 2;
      left_toe_vel_ref << 0, 0, 0;
      left_toe_acc_ref << 0, 0, 0;
    }

    if(contact(1) == 0){
      // temporally use capture point
      double x_goal = pel_pos(0) + -0.07 + cp_py * sqrt(.9/9.81) * pel_vel(0) + vel_des_x * 0;
      double y_goal = min(pel_pos(1) - 0.1 + cp_py * sqrt(.9/9.81) * pel_vel(1), (left_toe_pos(1) + left_toe_back_pos(1))/2 - 0.02);

      if(use_cap == 0){
        VectorXd foot_cmd = mpc_cmd_listener.get_right_foot_cmd();
        x_goal = foot_cmd(0) + pel_pos(0);
        y_goal = foot_cmd(1) + pel_pos(1);
      }
      
      double w = M_PI / (step_time - ds_time);
      double n = traj_time - ds_time / 2;
      if(traj_time - ds_time/2 < 0.005)
        foot_start << (right_toe_pos(0) + right_toe_back_pos(0))/2, (right_toe_pos(1) + right_toe_back_pos(1))/2;
      
      double dy_goal  = .5 * y_goal * w * (sin(w * n)) + .5 * w * (-sin(w * n)) * foot_start(1);
      double ddy_goal = .5 * y_goal * w * w * (cos(w * n)) + .5 * w * w * (-cos(w * n)) * foot_start(1);
      y_goal   = .5 * y_goal * (1 - cos(min(mpc_py * w * n,M_PI))) + .5 * (1 + cos(min(mpc_py * w * n,M_PI))) * foot_start(1);

      double dx_goal  = .5 * x_goal * w * (sin(w * n)) + .5 * w * (-sin(w * n)) * foot_start(0);
      double ddx_goal = .5 * x_goal * w * w * (cos(w * n)) + .5 * w * w * (-cos(w * n)) * foot_start(0);
      x_goal   = .5 * x_goal * (1 - cos(min(mpc_px * w * n,M_PI))) + .5 * (1 + cos(min(mpc_px * w * n,M_PI))) * foot_start(0);

      right_toe_pos_ref << x_goal, min(y_goal,(left_toe_pos(1) + left_toe_back_pos(1))/2 - 0.05), 0;
      right_toe_vel_ref << dx_goal, dy_goal, 0;
      right_toe_acc_ref << ddx_goal,ddy_goal,0;

      if(traj_time < step_time/2){
        double n = traj_time - ds_time / 2;
        tvec   << 1,n,pow(n,2),pow(n,3),pow(n,4),pow(n,5);
        dtvec  << 0,1,2*n,3*pow(n,2),4*pow(n,3),5*pow(n,4);
        ddtvec << 0,0,2,6*n,12*pow(n,2),20*pow(n,3);
        right_toe_pos_ref(2) = a.dot(tvec) - (pos_avg(2) - (left_toe_pos(2) + left_toe_back_pos(2))/2) - 0.0; // reset the ground level to current stance foot position
        right_toe_vel_ref(2) = a.dot(dtvec);
        right_toe_acc_ref(2) = a.dot(ddtvec);
      }
      else{
        double n = traj_time - step_time/2;
        tvec   << 1,n,pow(n,2),pow(n,3),pow(n,4),pow(n,5);
        dtvec  << 0,1,2*n,3*pow(n,2),4*pow(n,3),5*pow(n,4);
        ddtvec << 0,0,2,6*n,12*pow(n,2),20*pow(n,3);
        right_toe_pos_ref(2) = b.dot(tvec) - (pos_avg(2) - (left_toe_pos(2) + left_toe_back_pos(2))/2) - 0.0; // reset the ground level to current stance foot position
        right_toe_vel_ref(2) = b.dot(dtvec);
        right_toe_acc_ref(2) = b.dot(ddtvec);
      }
    }
    else{
      right_toe_pos_ref = (right_toe_pos + right_toe_back_pos) / 2;
      right_toe_vel_ref << 0, 0, 0;
      right_toe_acc_ref << 0, 0, 0;
    }

    VectorXd des_acc = VectorXd::Zero(6,1);
    des_acc << -KP_ToeF(0) * (left_toe_pos(0) - left_toe_pos_ref(0)   - 0.08) - KD_ToeF(0) * (left_toe_vel(0) - left_toe_vel_ref(0)) + left_toe_acc_ref(0),
               -KP_ToeF(1) * (left_toe_pos(1) - left_toe_pos_ref(1))  - KD_ToeF(1) * (left_toe_vel(1) - left_toe_vel_ref(1)) + left_toe_acc_ref(1), 
               -KP_ToeF(2) * (left_toe_pos(2) - left_toe_pos_ref(2))  - KD_ToeF(2) * (left_toe_vel(2) - left_toe_vel_ref(2)) + left_toe_acc_ref(2),
               -KP_ToeF(3) * (right_toe_pos(0) - right_toe_pos_ref(0) - 0.08) - KD_ToeF(3) * (right_toe_vel(0) - right_toe_vel_ref(0)) + right_toe_acc_ref(0),
               -KP_ToeF(4) * (right_toe_pos(1) - right_toe_pos_ref(1))- KD_ToeF(4) * (right_toe_vel(1) - right_toe_vel_ref(1)) + right_toe_acc_ref(1), 
               -KP_ToeF(5) * (right_toe_pos(2) - right_toe_pos_ref(2))- KD_ToeF(5) * (right_toe_vel(2) - right_toe_vel_ref(2)) + right_toe_acc_ref(2);

    VectorXd des_acc_toe = VectorXd::Zero(8,1);
    // Currently, need the forth dimension to control leg yaw rotation
    des_acc_toe << -KP_ToeB(0) * (left_toe_rot(0) - left_toe_pos_ref(0) + 0.08) - KD_ToeB(0) * (left_toe_drot(0) - left_toe_vel_ref(0)) + left_toe_acc_ref(0),
                   -KP_ToeB(1) * (left_toe_rot(1) - left_toe_pos_ref(1)) - KD_ToeB(1) * (left_toe_drot(1) - left_toe_vel_ref(1)) + left_toe_acc_ref(1),
                   -KP_ToeB(2) * (left_toe_rot(2) - left_toe_pos_ref(2)) - KD_ToeB(2) * (left_toe_drot(2) - left_toe_vel_ref(2)) + left_toe_acc_ref(2),
                   -cphy * (left_toe_rot(3) - theta(2)) - cdhy * (left_toe_drot(3) - 0),
                   -KP_ToeB(3) * (right_toe_rot(0) - right_toe_pos_ref(0) + 0.08) - KD_ToeB(3) * (right_toe_drot(0) - right_toe_vel_ref(0)) + right_toe_acc_ref(0),
                   -KP_ToeB(4) * (right_toe_rot(1) - right_toe_pos_ref(1)) - KD_ToeB(4) * (right_toe_drot(1) - right_toe_vel_ref(1)) + right_toe_acc_ref(1),
                   -KP_ToeB(5) * (right_toe_rot(2) - right_toe_pos_ref(2)) - KD_ToeB(5) * (right_toe_drot(2) - right_toe_vel_ref(2)) + right_toe_acc_ref(2),
                   -cphy * (right_toe_rot(3) - theta(2)) - cdhy * (right_toe_drot(3) - 0);
    
    // Zero accelerations for stance foot
    if(contact(0) != 0){
      des_acc.block(0,0,3,1) = VectorXd::Zero(3,1);
      des_acc_toe.block(0,0,3,1) = VectorXd::Zero(3,1); 
    }

    if(contact(1) != 0){
      des_acc.block(3,0,3,1) = VectorXd::Zero(3,1);
      des_acc_toe.block(4,0,3,1) = VectorXd::Zero(3,1);
    }

    // B matrix
    MatrixXd B = get_B(wb_q);

    // Spring Jacobian
    MatrixXd Spring_Jaco = get_Spring_Jaco();

    // For pelvis control in standing OSC
    VectorXd des_acc_pel = VectorXd::Zero(6,1);
    MatrixXd pel_jaco = MatrixXd::Zero(6,20);
    pel_jaco.block(0,0,6,6) = MatrixXd::Identity(6,6);

    if(contact(0) == 1 && contact(1) == 1){
      des_acc_pel << -KP_pel(0) * (pel_pos(0) - pel_pos_des(0)) - KD_pel(0) * (pel_vel(0) - pel_vel_des(0)),
                     -KP_pel(1) * (pel_pos(1) - pel_pos_des(1)) - KD_pel(1) * (pel_vel(1) - pel_vel_des(1)),
                     -KP_pel(2) * (pel_pos(2) - pel_pos_des(2)) - KD_pel(2) * (pel_vel(2) - pel_vel_des(2)),
                     -KP_pel(3) * (theta(2) - 0) - KD_pel(3) * (dtheta(2) - pel_omg_des(2)),
                     -KP_pel(4) * (theta(1) - 0) - KD_pel(4) * (dtheta(1) - pel_omg_des(1)),
                     -KP_pel(5) * (theta(0) - 0) - KD_pel(5) * (dtheta(0) - pel_omg_des(0));
    }
    else if(contact(0) == 0 || contact(1) == 0){
      VectorXd foot = pel_pos;
      if(contact(0) == 0)
        foot = .5 * (right_toe_back_pos + right_toe_pos);
      if(contact(1) == 0)
        foot = .5 * (left_toe_back_pos + left_toe_pos);
      des_acc_pel << -2.2 * KP_pel(0) * (pel_pos(0) - pel_pos_des(0)) - 2.2 * KD_pel(0) * (pel_vel(0) - pel_vel_des(0)) - fcdx * (pel_vel(0) - vel_des_x),
                     -2.2 * KP_pel(1) * (pel_pos(1) - pel_pos_des(1)) - 2.2 * KD_pel(1) * (pel_vel(1) - pel_vel_des(1)) - fcdy * (pel_vel(1) - vel_des_y),
                     -2 * KP_pel(2) * (pel_pos(2) - pel_pos_des(2)) - 2 * KD_pel(2) * (pel_vel(2) - pel_vel_des(2)),
                     -KP_pel(3) * (theta(2) - 0) - KD_pel(3) * (dtheta(2) - pel_omg_des(2)),
                     -KP_pel(4) * (theta(1) - 0) - KD_pel(4) * (dtheta(1) - pel_omg_des(1)),
                     -KP_pel(5) * (theta(0) - 0) - KD_pel(5) * (dtheta(0) - pel_omg_des(0));
    }
    else{
      des_acc_pel << - fcdx * (pel_vel(0) - vel_des_x),
                     - fcdy * (pel_vel(1) - vel_des_y),
                     -2 * KP_pel(2) * (pel_pos(2) - pel_pos_des(2)) - 2 * KD_pel(2) * (pel_vel(2) - pel_vel_des(2)),
                     -KP_pel(3) * (theta(2) - 0) - KD_pel(3) * (dtheta(2) - pel_omg_des(2)),
                     -KP_pel(4) * (theta(1) - 0) - KD_pel(4) * (dtheta(1) - pel_omg_des(1)),
                     -KP_pel(5) * (theta(0) - 0) - KD_pel(5) * (dtheta(0) - pel_omg_des(0));
    }

    
    elapsed_time = duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - time_program_start);
    //cout << "time used to compute system dyn and kin + acc + QP Form: " << elapsed_time.count() << endl;
    // Solve OSC QP
    elapsed_time = duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - time_program_start);
    //cout << "set up sparse constraint: " << elapsed_time.count() << endl;
    if(QP_initialized == 0){
      QP_initialized = 1;
      bool check_solver = false; // change this to false if you want to check OSQP solver output for debug
      if(osc_version == 0){
        osc.setupQPVector(des_acc_pel, des_acc, des_acc_toe, G, contact);
        osc.setupQPMatrix(Weight_pel, M, 
                          B, Spring_Jaco, left_toe_jaco_fa, 
                          left_toe_back_jaco_fa, right_toe_jaco_fa, right_toe_back_jaco_fa,
                          left_toe_rot_jaco_fa, right_toe_rot_jaco_fa);
        osc.setUpQP(check_solver); 
      }
      else{
        oscV2.setupQPVector(des_acc_pel, des_acc, des_acc_toe, G, contact);
        oscV2.setupQPMatrix(Weight_pel, M, 
                          B, Spring_Jaco, left_toe_jaco_fa, 
                          left_toe_back_jaco_fa, right_toe_jaco_fa, right_toe_back_jaco_fa,
                          left_toe_rot_jaco_fa, right_toe_rot_jaco_fa);
        oscV2.setUpQP(check_solver); 
      }
    }
    else{
      if(update_mat == -1){
        if(osc_version == 0){
          osc.updateQPVector(des_acc_pel, des_acc, des_acc_toe, G + D_term, contact);
          osc.updateQPMatrix(Weight_pel, M, 
                            B, Spring_Jaco, left_toe_jaco_fa, 
                            left_toe_back_jaco_fa, right_toe_jaco_fa, right_toe_back_jaco_fa,
                            left_toe_rot_jaco_fa, right_toe_rot_jaco_fa, contact);
          elapsed_time = duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - time_program_start);
          //cout << "set up QP matrix: " << elapsed_time.count() << endl;
          osc.updateQP();
        }
        else{
          oscV2.updateQPVector(des_acc_pel, des_acc, des_acc_toe, G + D_term, contact);
          oscV2.updateQPMatrix(Weight_pel, M, 
                            B, Spring_Jaco, left_toe_jaco_fa, 
                            left_toe_back_jaco_fa, right_toe_jaco_fa, right_toe_back_jaco_fa,
                            left_toe_rot_jaco_fa, right_toe_rot_jaco_fa, contact);
          elapsed_time = duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - time_program_start);
          //cout << "set up QP matrix: " << elapsed_time.count() << endl;
          oscV2.updateQP();
        }
      }
    }

    //solver.solveProblem();
    //QPSolution = solver.getSolution();
    if(update_mat == -1){
      if(osc_version == 0){
        QPSolution = osc.solveQP();
      }
      else{
        QPSolution = oscV2.solveQP();
      }
    }
    //update_mat *= -1;
    VectorXd torque = VectorXd::Zero(12,1);
    for(int i = 0;i<12;i++)
      torque(i) = QPSolution(20+i);
    

    // OSC on leg, PD on arm
    elapsed_time = duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - time_program_start);
    //cout << "time used to compute system dyn and kin + QP formulation + Solving: " << elapsed_time.count() << endl;

    // Integrate osc ddq with trapezoidal rule to find velocity command
    VectorXd wb_dq_next = VectorXd::Zero(12,1);
    for(int i=0;i<12;i++){
      if(i<4) 
        wb_dq_next(i) = wb_dq(6+i) + damping_dt * 0.5 * (QPSolution(6+i) + last_QPSolution(6+i)); // skip passive joint
      else if(i<10)
        wb_dq_next(i) = wb_dq(7+i) + damping_dt * 0.5 * (QPSolution(7+i) + last_QPSolution(6+i));
      else
        wb_dq_next(i) = wb_dq(8+i) + damping_dt * 0.5 * (QPSolution(8+i) + last_QPSolution(6+i));
    }
    last_QPSolution = QPSolution;

    
    wb_dq_next(2) = 0*wb_dq_next(2);
    wb_dq_next(3) = 1*wb_dq_next(3);
    wb_dq_next(8) = 0*wb_dq_next(8);
    wb_dq_next(9) = 1*wb_dq_next(9);

    if(contact(0) != 0){
      wb_dq_next.block(0,0,6,1) = VectorXd::Zero(6,1);
    }

    if(contact(1) != 0){
      wb_dq_next.block(6,0,6,1) = VectorXd::Zero(6,1);
    }

    // Foot joint velocity command
    wb_dq_next(4) = 0;
    wb_dq_next(5) = 0;
    wb_dq_next(10) = 0;
    wb_dq_next(11) = 0;
    
    
    // IK arm control for conducting, trial implementation. Incorporate to analytical_expressions class in the future
    VectorXd ql = VectorXd::Zero(10,1);
    VectorXd p_lh = VectorXd::Zero(3,1);
    MatrixXd J_lh = MatrixXd::Zero(3,10);
    VectorXd p_lh_ref = VectorXd::Zero(3,1);
    VectorXd p_lh_err = VectorXd::Zero(3,1);

    VectorXd qr = VectorXd::Zero(10,1);
    VectorXd p_rh = VectorXd::Zero(3,1);
    MatrixXd J_rh = MatrixXd::Zero(3,10);
    VectorXd p_rh_ref = VectorXd::Zero(3,1);
    VectorXd p_rh_err = VectorXd::Zero(3,1);

    double cur_time = (observation.time - digit_time_start);
    double period = arm_z_prd;

    // The IK is always in the base frame
    if(stepping == 2){
      if(stance_leg == 1){
        p_lh_ref << 0.02 + 0.1 * sin(M_PI * traj_time/step_time), + 0.25,  + arm_z_int;
        p_rh_ref << 0.02 - 0.1 * sin(M_PI * traj_time/step_time), - 0.25,  + arm_z_int;
      }
      else{
        p_lh_ref << 0.02 - 0.1 * sin(M_PI * traj_time/step_time), + 0.25,  + arm_z_int;
        p_rh_ref << 0.02 + 0.1 * sin(M_PI * traj_time/step_time), - 0.25,  + arm_z_int;
      }
    }
    else{
      p_lh_ref << + 0.02, + 0.25, + arm_z_int;
      p_rh_ref << + 0.02, - 0.25, + arm_z_int;
    }

    // initialize q with current pose
    ql.block(6,0,4,1) = wb_q.block(20,0,4,1);
    qr.block(6,0,4,1) = wb_q.block(24,0,4,1);
    kin_left_arm(ql.data(),p_lh.data(), J_lh.data());
    kin_right_arm(qr.data(),p_rh.data(), J_rh.data());
    J_lh.block(0,0,3,7) = MatrixXd::Zero(3,7); // base is fixed for arm IK
    J_rh.block(0,0,3,7) = MatrixXd::Zero(3,7); // base is fixed for arm IK
     
    
    // left arm IK
    double error;
    error = (p_lh - p_lh_ref).norm();
    double iter = 0;

    // IK is warmed started with last solution is goal is close enough. Otherwise initialize with current configuration
    if(error > 0.01){
      while(error > 0.01 && iter<2){
        // solve for new joint
        ql += J_lh.colPivHouseholderQr().solve(p_lh_ref - p_lh);
        // Clip joints
        ql(6) = max(min(ql(6),deg2rad(75)),deg2rad(-75));
        ql(7) = max(min(ql(7),deg2rad(145)),deg2rad(-145));
        ql(8) = max(min(ql(8),deg2rad(100)),deg2rad(-100));
        ql(9) = max(min(ql(9),deg2rad(77.5)),deg2rad(-77.5));
        // Evaluate new pos
        kin_left_arm(ql.data(),p_lh.data(), J_lh.data());
        J_lh.block(0,0,3,7) = MatrixXd::Zero(3,7); // base is fixed for arm IK
        error = (p_lh - p_lh_ref).norm();
        iter++;
      }
      target_position[12] = ql(6);
      target_position[13] = ql(7);
      target_position[14] = ql(8);
      target_position[15] = ql(9); 
      ql_last = ql;
    }
    else{
      ql = ql_last;
    }
    // right arm IKpel_pos_des
    error = (p_rh - p_rh_ref).norm();
    iter = 0;
    if(error > 0.01){
      while(error > 0.01 && iter<2){
        qr += J_rh.colPivHouseholderQr().solve(p_rh_ref - p_rh);
        // Clip joints
        qr(6) = max(min(qr(6),deg2rad(75)),deg2rad(-75));
        qr(7) = max(min(qr(7),deg2rad(145)),deg2rad(-145));
        qr(8) = max(min(qr(8),deg2rad(100)),deg2rad(-100));
        qr(9) = max(min(qr(9),deg2rad(77.5)),deg2rad(-77.5));
        // Evaluate new pos
        kin_right_arm(qr.data(),p_rh.data(), J_rh.data());
        J_rh.block(0,0,3,7) = MatrixXd::Zero(3,7); // base is fixed for arm IK
        error = (p_rh - p_rh_ref).norm();
        iter++;
      }
      target_position[16] = qr(6);     
      target_position[17] = qr(7);
      target_position[18] = qr(8);
      target_position[19] = qr(9); 
      qr_last = qr;
    }
    else{
      qr = qr_last;
    }
    

    // safety check
    safe_check.updateSafety(pb_q.block(6,0,14,1),pb_dq.block(6,0,14,1));
    elapsed_time = duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - time_program_start);
    //cout << "time used to compute system dyn and kin + QP formulation + Solving + Arm IK: " << elapsed_time.count() << endl;

    for (int i = 0; i < NUM_MOTORS; ++i) {
      if(safe_check.checkSafety()){ // safety check
          command.motors[i].torque = -arm_P/10 * observation.motor.velocity[i];
          command.motors[i].velocity = 0;
          command.motors[i].damping = 1 * limits->damping_limit[i];
          cout << "safety triggered" << endl;
      }
      else{
        // wrap up torque
        if(i>=12){
          command.motors[i].torque =
            min((observation.time - digit_time_start)/soft_count,1.0) * arm_P * (target_position[i] - observation.motor.position[i]);
            command.motors[i].velocity = 0;
            command.motors[i].damping = 0.75 * limits->damping_limit[i];
        }
        else{
          torque *= min((observation.time - digit_time_start)/soft_count,1.0);
          command.motors[i].torque = 1 * torque(i) ;
          command.motors[i].velocity = 1 * wb_dq_next(i);
          // Use different damping or velocity for stance and swing leg (Potentially better)
          if(i<6){
            if(contact(0) > 0)
              command.motors[i].damping = .2 * limits->damping_limit[i];
            else
              command.motors[i].damping = .2 * limits->damping_limit[i];
          }
          else{
            if(contact(1) > 0)
              command.motors[i].damping = .2 * limits->damping_limit[i];
            else
              command.motors[i].damping = .2 * limits->damping_limit[i];
          }
        }
      }
    }
    if(run_sim){
        command.fallback_opmode = Locomotion; // Useful for simulation
    }
    else{
        command.fallback_opmode = Damping;
    }
    command.apply_command = true;
    llapi_send_command(&command);

    // Get obstacle info
    VectorXd obs_pos = VectorXd(2,1);
    obs_pos << 5,5;
    VectorXd obs_tan = VectorXd(2,1);
    if(stepping == 3){
      // reset position command when walking direction is changed
      if(key_mode == 5){
        obs_pos << 0.2, 0.0;
      }
      if(key_mode == 6){
        obs_pos << -0.2, 0.0;
      }
      if(key_mode == 7){
        obs_pos << 0.0, 0.2;
      }
      if(key_mode == 8){
        obs_pos << 0.0, -0.2;
      }      
    }

    obs_tan = -obs_pos / obs_pos.norm();
    // send mpc state info
    VectorXd pel_ref = VectorXd::Zero(4,1);      // x,y reference
    VectorXd st_foot_pos = VectorXd::Zero(2,1);  // stance foot position
    VectorXd obs_info = VectorXd::Zero(4,1);     // obstacle info
    pel_ref << 0.0, vel_des_x, 0.0, vel_des_y;              
    obs_info << obs_pos, obs_tan;           

    if(contact(0) == 0){
      st_foot_pos << (right_toe_pos(0) + right_toe_back_pos(0))/2 - pel_pos(0), (right_toe_pos(1) + right_toe_back_pos(1))/2 - pel_pos(1);
    }
    if(contact(1) == 0){
      st_foot_pos << (left_toe_pos(0) + left_toe_back_pos(0))/2  - pel_pos(0), (left_toe_pos(1) + left_toe_back_pos(1))/2 - pel_pos(1);
    }
    pel_pos(0) = 0;
    pel_pos(1) = 0;
    Digit_Ros::digit_state msg;
    // MPC data
    std::copy(pel_pos.data(),pel_pos.data() + 3,msg.pel_pos.begin());
    std::copy(pel_vel.data(),pel_vel.data() + 3,msg.pel_vel.begin());
    std::copy(theta.data(),theta.data() + 3,msg.pel_rot.begin());
    std::copy(dtheta.data(),dtheta.data() + 3,msg.pel_omg.begin());
    std::copy(pel_ref.data(),pel_ref.data() + 4, msg.pel_ref.begin());
    std::copy(st_foot_pos.data(),st_foot_pos.data() + 2,msg.foot_pos.begin());
    std::copy(obs_info.data(),obs_info.data() + 4,msg.obs_info.begin());

    // OSC data for visualization
    std::copy(pel_pos_des.data(), pel_pos_des.data() + pel_pos_des.size(), msg.pel_pos_des.begin());
    std::copy(pel_vel_des.data(), pel_vel_des.data() + pel_vel_des.size(), msg.pel_vel_des.begin());
    std::copy(left_toe_pos.data(), left_toe_pos.data() + left_toe_pos.size(), msg.left_toe_pos.begin());
    std::copy(left_toe_pos_ref.data(), left_toe_pos_ref.data() + left_toe_pos_ref.size(), msg.left_toe_pos_ref.begin());
    std::copy(left_toe_vel.data(), left_toe_vel.data() + left_toe_vel.size(), msg.left_toe_vel.begin());
    std::copy(left_toe_vel_ref.data(), left_toe_vel_ref.data() + left_toe_vel_ref.size(), msg.left_toe_vel_ref.begin());
    std::copy(right_toe_pos.data(), right_toe_pos.data() + right_toe_pos.size(), msg.right_toe_pos.begin());
    std::copy(right_toe_pos_ref.data(), right_toe_pos_ref.data() + right_toe_pos_ref.size(), msg.right_toe_pos_ref.begin());
    std::copy(right_toe_vel.data(), right_toe_vel.data() + right_toe_vel.size(), msg.right_toe_vel.begin());
    std::copy(right_toe_vel_ref.data(), right_toe_vel_ref.data() + right_toe_vel_ref.size(), msg.right_toe_vel_ref.begin());
    std::copy(torque.data(), torque.data() + torque.size(), msg.torque.begin());
    std::copy(torq.data(), torq.data() + torq.size(), msg.torq.begin());
    std::copy(contact.data(), contact.data() + contact.size(), msg.contact.begin());
    msg.traj_time = traj_time;
    msg.stance_leg = stance_leg;

    if(recording)
      bag.write("digit_state", ros::Time::now(), msg);

      //msg.yaw = yaw_des;
    state_pub.publish(msg);
    ros::spinOnce();
    control_loop_rate.sleep();
    //cout << "Desired yaw angle is: " << msg.yaw << endl;
    // Check if llapi has become disconnected
    if(ros::isShuttingDown()){
      if(recording)
        bag.close();
      break;
    }
    if (!llapi_connected()) {
      // Handle error case. You don't need to re-initialize subscriber
      // Calling llapi_send_command will keep low level api open
    }

    // Sleep to keep to a reasonable update rate
    //usleep(2000);
    /*int time = 1000-elapsed_time.count();
    int sleep_time = max(0,time);
    usleep(sleep_time);*/
  }
}

MatrixXd get_Spring_Jaco(){
  MatrixXd Spring_Jaco = MatrixXd::Zero(4,20);
  Spring_Jaco(0,wbc::left_knee) = 1;
  Spring_Jaco(0,wbc::left_tarsus) = 1; 
  Spring_Jaco(1,wbc::left_toe_pitch) = 0; 
  Spring_Jaco(1,wbc::left_toe_roll) = 0; 

  Spring_Jaco(2,wbc::right_knee) = 1;
  Spring_Jaco(2,wbc::right_tarsus) = 1; 
  Spring_Jaco(3,wbc::right_toe_pitch) = 0; 
  Spring_Jaco(3,wbc::right_toe_roll) = 0; 

  return Spring_Jaco;
}

MatrixXd get_B(VectorXd q){
  // map: motor torque to joint torque
  MatrixXd B = MatrixXd::Zero(20,12);
  VectorXd a,b,c,d;
  a.resize(6);
  b.resize(6);
  c.resize(6);
  d.resize(6);
  a << 0.003061, -0.9412, 0.2812, -0.1121, 0.1288, 0.06276; // higher order approximation?
  b << -0.003154, 0.9416, 0.2848, 0.1133, 0.1315, -0.06146;
  c << -0.003061, -0.9412, 0.2812, 0.1121, -0.1288, -0.06276;
  d << 0.003154, 0.9416, 0.2848, -0.1133, -0.1315, 0.06146;

  // Replace left_J and right_J with left_J2 and right_J2 for a better approximation
  /*e.resize(15);
  f.resize(15);
  g.resize(15);
  h.resize(15);

  e <<  0.01785,-0.9256,0.2938,-0.08362,0.103,0.06534,0.02975,-0.02949,-0.01311,-0.03942,-0.03918,0.06356,-0.0451,-0.02977,-0.003042;  
  f << -0.01785,0.9257,0.2972,0.08384,0.1044,-0.06483,-0.02988,-0.02979,0.01411,-0.039,0.04013,0.06584,0.04692,-0.02893,0.003069; 
  g << -0.01785,-0.9255,0.2938,0.08367,-0.1029,-0.06529,0.0297,-0.02936,-0.01315,-0.03937,0.03896,-0.06342,0.04496,0.02929,0.002823;  
  h <<  0.01785,0.9257,0.2972,-0.08391,-0.1045,0.06483,-0.02982,-0.02973,0.01419,-0.03903,-0.03976,-0.06553,-0.04701,0.02931,-0.003061; 

  MatrixXd left_J2 = get_pr2m_jaco(e,f,q(11),q(12));
  MatrixXd right_J2 = get_pr2m_jaco(g,h,q(18),q(19));
  */
  VectorXd left_toe_j = VectorXd::Zero(2,1);
  MatrixXd left_J = MatrixXd::Zero(2,2);
  left_toe_j << q(11) , q(12);


  left_J(0,0) = a(1) + 2*a(3) * left_toe_j(0) + a(4) * left_toe_j(1);
  left_J(0,1) = a(2) + 2*a(5) * left_toe_j(1) + a(4) * left_toe_j(0); 
  left_J(1,0) = b(1) + 2*b(3) * left_toe_j(0) + b(4) * left_toe_j(1);
  left_J(1,1) = b(2) + 2*b(5) * left_toe_j(1) + b(4) * left_toe_j(0);

  B(wbc::left_hip_roll,0) = 1;
  B(wbc::left_hip_yaw,1) = 1;
  B(wbc::left_hip_pitch,2) = 1;
  B(wbc::left_knee,3) = 1;
  //B(wbc::left_toe_pitch,4) = 1;
  //B(wbc::left_toe_roll,5) = 1;

  B(wbc::left_toe_pitch,4) = left_J(0,0);
  B(wbc::left_toe_pitch,5) = left_J(1,0);
  B(wbc::left_toe_roll,4) = left_J(0,1);
  B(wbc::left_toe_roll,5) = left_J(1,1);

  VectorXd right_toe_j = VectorXd::Zero(2,1);
  MatrixXd right_J = MatrixXd::Zero(2,2);
  right_toe_j << q(18) , q(19);

  right_J(0,0) = c(1) + 2*c(3) * right_toe_j(0) + c(4) * right_toe_j(1);
  right_J(0,1) = c(2) + 2*c(5) * right_toe_j(1) + c(4) * right_toe_j(0); 
  right_J(1,0) = d(1) + 2*d(3) * right_toe_j(0) + d(4) * right_toe_j(1);
  right_J(1,1) = d(2) + 2*d(5) * right_toe_j(1) + d(4) * right_toe_j(0);

  B(wbc::right_hip_roll,6) = 1;
  B(wbc::right_hip_yaw,7) = 1;
  B(wbc::right_hip_pitch,8) = 1;
  B(wbc::right_knee,9) = 1;
  //B(wbc::right_toe_pitch,10) = 1;
  //B(wbc::right_toe_roll,11) = 1;

  B(wbc::right_toe_pitch,10) = right_J(0,0);
  B(wbc::right_toe_pitch,11) = right_J(1,0);
  B(wbc::right_toe_roll,10) = right_J(0,1);
  B(wbc::right_toe_roll,11) = right_J(1,1);
  return B;
}

MatrixXd get_fric_constraint(double mu){
  MatrixXd fric_cons = MatrixXd::Zero(16,12);
  MatrixXd fric_sub  = MatrixXd::Zero(4,3);
  fric_sub << 1,1,-mu,1,-1,-mu,-1,1,-mu,-1,-1,-mu;
  for(int i=0;i<4;i++){
    fric_cons.block(4*i,3*i,4,3) = fric_sub;
  }
  return fric_cons;
}

VectorXd ToEulerAngle(VectorXd q){
    VectorXd theta = VectorXd::Zero(3,1);
    double w,x,y,z;
    w = q(0);
    x = q(1);
    y = q(2);
    z = q(3);

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    theta(0) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1) {
      theta(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    } else {
      theta(1) = std::asin(sinp);
    }
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    theta(2) = std::atan2(siny_cosp, cosy_cosp);

    return theta;
}

MatrixXd get_pr2m_jaco(VectorXd a, VectorXd b, double x, double y){
  MatrixXd jaco = MatrixXd::Zero(2,2);
  jaco(0,0) = a(1) + 2*a(3)*x + a(4)*y + 3*a(6)*pow(x,2) + 2*a(7) * x*y + a(8)*pow(y,2) + 4*a(10)*pow(x,3) + 3*a(11)*pow(x,2)*y + 2*a(12)*x*pow(y,2) + a(13)*pow(y,3);
  jaco(0,1) = a(2) + 2*a(5)*y + a(4)*x + 3*a(9)*pow(y,2) + 2*a(8) * x*y + a(7)*pow(x,2) + 4*a(14)*pow(y,3) + 3*a(13)*pow(y,2)*x + 2*a(12)*y*pow(x,2) + a(11)*pow(x,3);
  jaco(1,0) = b(1) + 2*b(3)*x + b(4)*y + 3*b(6)*pow(x,2) + 2*b(7) * x*y + b(8)*pow(y,2) + 4*b(10)*pow(x,3) + 3*b(11)*pow(x,2)*y + 2*b(12)*x*pow(y,2) + b(13)*pow(y,3);
  jaco(1,1) = b(2) + 2*b(5)*y + b(4)*x + 3*b(9)*pow(y,2) + 2*b(8) * x*y + b(7)*pow(x,2) + 4*b(14)*pow(y,3) + 3*b(13)*pow(y,2)*x + 2*b(12)*y*pow(x,2) + b(11)*pow(x,3);
  return jaco;
}

VectorXd get_quintic_params(VectorXd x0, VectorXd xT, double T){
  MatrixXd A = MatrixXd::Zero(6,6);
  VectorXd b(6); b << x0, xT;

  A(0,0) = 1;
  A(1,1) = 1;
  A(2,2) = 2;
  A.block(3,0,1,6) << 1, T, pow(T,2), pow(T,3), pow(T,4), pow(T,5);
  A.block(4,1,1,5) << 1, 2*T,  3*pow(T,2), 4*pow(T,3), 5*pow(T,4);
  A.block(5,2,1,4) << 2, 6*T, 12*pow(T,2), 20*pow(T,3);

  VectorXd sol = A.colPivHouseholderQr().solve(b);
  return sol;
}