#include "mpc_listener.hpp"
#include "utilities.hpp"

using namespace Eigen;
using namespace std;

MPC_CMD_Listener::MPC_CMD_Listener()
{
  mpc_cmd_sub = node_handler.subscribe("mpc_res", 10, &MPC_CMD_Listener::MPCInputCallback, this);
  pel_pos_cmd = VectorXd::Zero(4,1);
  pel_vel_cmd = VectorXd::Zero(4,1);
  left_foot_cmd = VectorXd::Zero(3,1);
  right_foot_cmd = VectorXd::Zero(3,1);
  swing_foot_cmd = VectorXd::Zero(15,1);
}

void MPC_CMD_Listener::MPCInputCallback(const Digit_Ros::mpc_info& msg) {
  for(int i = 0;i<3;i++){
    left_foot_cmd[i]   = msg.foot_left_cmd[i];
    right_foot_cmd[i]  = msg.foot_right_cmd[i];
  }
  std::copy(msg.pel_pos_cmd.begin(), msg.pel_pos_cmd.begin() + 4, pel_pos_cmd.data());
  std::copy(msg.pel_vel_cmd.begin(), msg.pel_vel_cmd.begin() + 4, pel_vel_cmd.data());
  std::copy(msg.swing_foot_cmd.begin(), msg.swing_foot_cmd.begin() + 15, swing_foot_cmd.data());
}
