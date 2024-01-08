#include "mpc_listener.hpp"
#include "utilities.hpp"

using namespace Eigen;
using namespace std;

MPC_CMD_Listener::MPC_CMD_Listener()
{
  mpc_cmd_sub = node_handler.subscribe("mpc_res", 10, &MPC_CMD_Listener::MPCInputCallback, this);
  pel_pos_cmd = VectorXd::Zero(3,1);
  pel_vel_cmd = VectorXd::Zero(3,1);
  left_foot_cmd = VectorXd::Zero(3,1);
  right_foot_cmd = VectorXd::Zero(3,1);
}

void MPC_CMD_Listener::MPCInputCallback(const Digit_Ros::mpc_info& msg) {
  for(int i = 0;i<3;i++){
    pel_pos_cmd[i] = msg.pel_pos_cmd[i]; 
    pel_vel_cmd[i] = msg.pel_vel_cmd[i];
    left_foot_cmd[i]   = msg.foot_left_cmd[i];
    right_foot_cmd[i]  = msg.foot_right_cmd[i];
  }
}
