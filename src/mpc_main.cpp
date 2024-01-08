#include "mpc_main.hpp"
#include "utilities.hpp"

using namespace Eigen;
using namespace std;

Digit_MPC::Digit_MPC()
{
  mpc_sub = node_handler.subscribe("/digit_state", 10, &Digit_MPC::MPCInputCallback, this);
  pel_pos = VectorXd::Zero(3,1);
  pel_vel = VectorXd::Zero(3,1);
  theta = VectorXd::Zero(3,1);
  dtheta = VectorXd::Zero(3,1);
}

void Digit_MPC::MPCInputCallback(const Digit_Ros::digit_state& msg) {
  for(int i = 0;i<3;i++){
    pel_pos[i] = msg.pel_pos[i]; 
    pel_vel[i] = msg.pel_vel[i];
    theta[i]   = msg.pel_rot[i];
    dtheta[i]  = msg.pel_omg[i];
  }
  cout << "updated mpc measurement:" << endl;
  cout << pel_pos[0] << endl;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "listener");
  Digit_MPC digit_mpc;

  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  ros::Publisher mpc_res_pub = n.advertise<Digit_Ros::mpc_info>("mpc_res", 1000);
  int count = 0;

  while (ros::ok()){

    Digit_Ros::mpc_info msg;
    for(int i = 0;i<3;i++){
      msg.pel_pos_cmd[i] = -3.14 * count;
      msg.pel_vel_cmd[i] = -3.14 * count;
      msg.foot_left_cmd[i] = -1.14 * count;
      msg.foot_right_cmd[i] = -1.14 * count;
    }
    
    cout << "mpc cmd published is: " << endl;
    cout << msg.pel_pos_cmd[0] << endl;

    
    mpc_res_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
    if(count >=60)
      count = 0;
  }
  //ros::Subscriber sub = n.subscribe("/digit_state", 100, chatterCallback);
  ros::spin();
  return 0;
  //ros::spin();
}