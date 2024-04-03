#include "mpc_mainV2.hpp"
#include "utilities.hpp"

using namespace Eigen;
using namespace std;
namespace fs = std::filesystem;
using namespace std::chrono;

Digit_MPC::Digit_MPC()
{
  mpc_sub_ = node_handler_.subscribe("/digit_state", 10, &Digit_MPC::MPCInputCallback, this);
  pel_pos_ = VectorXd::Zero(3,1);
  pel_vel_ = VectorXd::Zero(3,1);
  theta_ = VectorXd::Zero(3,1);
  dtheta_ = VectorXd::Zero(3,1);
  pel_ref_ = VectorXd::Zero(4,1);
  foot_pos_ = VectorXd::Zero(2,1);
  obs_info_ = VectorXd::Zero(4,1);
  traj_time_ = 0;
  stance_leg_ = 1;

  // MPC QP Matrix
  std::string prefix_lib = fs::current_path().parent_path().string();
  left_step0_matrix_ = casadi::external("LeftStart_Step0", prefix_lib + "/catkin_ws/src/Digit_Main/mpc_lib/LeftStartQP_Step0_04Tra.so");
  left_step1_matrix_ = casadi::external("LeftStart_Step1", prefix_lib + "/catkin_ws/src/Digit_Main/mpc_lib/LeftStartQP_Step1_04Tra.so");
  left_step2_matrix_ = casadi::external("LeftStart_Step2", prefix_lib + "/catkin_ws/src/Digit_Main/mpc_lib/LeftStartQP_Step2_04Tra.so");
  left_step3_matrix_ = casadi::external("LeftStart_Step3", prefix_lib + "/catkin_ws/src/Digit_Main/mpc_lib/LeftStartQP_Step3_04Tra.so");

  right_step0_matrix_ = casadi::external("RightStart_Step0", prefix_lib + "/catkin_ws/src/Digit_Main/mpc_lib/RightStartQP_Step0_04Tra.so");
  right_step1_matrix_ = casadi::external("RightStart_Step1", prefix_lib + "/catkin_ws/src/Digit_Main/mpc_lib/RightStartQP_Step1_04Tra.so");
  right_step2_matrix_ = casadi::external("RightStart_Step2", prefix_lib + "/catkin_ws/src/Digit_Main/mpc_lib/RightStartQP_Step2_04Tra.so");
  right_step3_matrix_ = casadi::external("RightStart_Step3", prefix_lib + "/catkin_ws/src/Digit_Main/mpc_lib/RightStartQP_Step3_04Tra.so");

  // read control parameters
  std::string package_path; 
  try {
    package_path = ros::package::getPath("Digit_Ros");
    if (package_path.empty()) {
      throw 1;
    }
  } catch(...) {
    std::cerr << "package not found\n";
  }
  std::shared_ptr<cpptoml::table> config = cpptoml::parse_file(package_path + "/src/config_file/mpc_robot_config.toml");
  double flx = config->get_qualified_as<double>("MPC-Params.foot_x_max").value_or(0);
  double fly = config->get_qualified_as<double>("MPC-Params.foot_y_max").value_or(0);
  double Wx = config->get_qualified_as<double>("MPC-Params.W_com_x_p").value_or(0);
  double Wdx = config->get_qualified_as<double>("MPC-Params.W_com_x_d").value_or(0);
  double Wy = config->get_qualified_as<double>("MPC-Params.W_com_y_p").value_or(0);
  double Wdy = config->get_qualified_as<double>("MPC-Params.W_com_y_d").value_or(0);
  double Wux = config->get_qualified_as<double>("MPC-Params.W_foot_ux").value_or(0);
  double Wuy = config->get_qualified_as<double>("MPC-Params.W_foot_uy").value_or(0);
  double Wdux = config->get_qualified_as<double>("MPC-Params.W_foot_dux").value_or(0);
  double Wduy = config->get_qualified_as<double>("MPC-Params.W_foot_duy").value_or(0);
  double Wobs = config->get_qualified_as<double>("MPC-Params.W_obs").value_or(0);
  double r1 = config->get_qualified_as<double>("MPC-Params.obs_rad1").value_or(0);
  double r2 = config->get_qualified_as<double>("MPC-Params.obs_rad2").value_or(0);

  std::shared_ptr<cpptoml::table> config_osc = cpptoml::parse_file(package_path + "/src/config_file/oscmpc_robot_config.toml");
  step_time_ = config_osc->get_qualified_as<double>("Walk-Params.step_time").value_or(0);
  ds_time_ = config_osc->get_qualified_as<double>("Walk-Params.ds_time").value_or(0);
  f_length_ = {flx, fly};
  Weights_ss_ = {Wx, Wdx, Wy, Wdy, Wux, Wuy, Wdux, Wduy, Wobs};
  Weights_ds_ = {0, 2500, 0, 2500, 10000, 10000, 100, 6000, 15000};
  r_ = {r1, r2};
  f_width_ = config->get_qualified_as<double>("MPC-Params.foot_width").value_or(0);

  // QP variable
  Nodes_ = 17;
  NPred_ = 4;
  nx_ = 2;
  
  // initialize solvers
  Cons_Num_ = {200,201,202,203};
  Vars_Num_ = 144;
  for(int i = 0; i<Vars_Num_;i++){
    sol_.push_back(0);
  }
  //
  mpc_solver0_ = MPC_Solver(Cons_Num_[0],Vars_Num_);
  mpc_solver1_ = MPC_Solver(Cons_Num_[1],Vars_Num_);
  mpc_solver2_ = MPC_Solver(Cons_Num_[2],Vars_Num_);
  mpc_solver3_ = MPC_Solver(Cons_Num_[3],Vars_Num_);
}

void Digit_MPC::MPCInputCallback(const Digit_Ros::digit_state& msg) {
  for(int i = 0;i<3;i++){
    pel_pos_[i] = msg.pel_pos[i]; 
    pel_vel_[i] = msg.pel_vel[i];
    theta_[i]   = msg.pel_rot[i];
    dtheta_[i]  = msg.pel_omg[i];
  }
  std::copy(msg.pel_ref.begin(), msg.pel_ref.begin() + 4, pel_ref_.data());
  std::copy(msg.foot_pos.begin(), msg.foot_pos.begin() + 2, foot_pos_.data());
  std::copy(msg.obs_info.begin(), msg.obs_info.begin() + 4, obs_info_.data());


  traj_time_ = msg.traj_time;
  stance_leg_ = msg.stance_leg;
}

VectorXd Digit_MPC::Update_MPC_(int traj_time, vector<vector<double>> mpc_input){
  VectorXd QPSolution;
  int mpc_in = traj_time;
  std::vector<double> input;
  input.insert(input.end(), mpc_input[0].begin(), mpc_input[0].end());
  input.insert(input.end(), mpc_input[1].begin(), mpc_input[1].end());
  input.insert(input.end(), f_length_.begin(), f_length_.end());
  input.insert(input.end(), mpc_input[2].begin(), mpc_input[2].end());
  input.insert(input.end(), mpc_input[3].begin(), mpc_input[3].end());
  input.insert(input.end(), Weights_ss_.begin(), Weights_ss_.end());
  input.insert(input.end(), r_.begin(), r_.end());
  input.insert(input.end(), mpc_input[4].begin(), mpc_input[4].end());
  input.insert(input.end(), mpc_input[5].begin(), mpc_input[5].end());
  input.insert(input.end(), mpc_input[6].begin(), mpc_input[6].end());
  std::vector<casadi::DM> MPC_arg = {input,sol_};
  std::vector<casadi::DM> res;

  if(stance_leg_ == -1){
      switch(mpc_in){
        case 0: res = left_step0_matrix_(MPC_arg); break;
        case 1: res = left_step1_matrix_(MPC_arg); break;
        case 2: res = left_step2_matrix_(MPC_arg); break;
        case 3: res = left_step3_matrix_(MPC_arg); break;
        default: ;
      }
  }
  else{
      switch(mpc_in){
        case 0: res = right_step0_matrix_(MPC_arg); break;
        case 1: res = right_step1_matrix_(MPC_arg); break;
        case 2: res = right_step2_matrix_(MPC_arg); break;
        case 3: res = right_step3_matrix_(MPC_arg); break;
        default: ;
      }
  }
  
  casadi::DM Aeq = res.at(0);
  casadi::DM beq = -res.at(1);
  casadi::DM Aiq = res.at(2);
  casadi::DM biq = -res.at(3);
  casadi::DM H = res.at(4);
  casadi::DM f = res.at(5);

  switch(mpc_in){
    case 0: QPSolution = mpc_solver0_.Update_Solver(Aeq,beq,Aiq,biq,H,f); break;
    case 1: QPSolution = mpc_solver1_.Update_Solver(Aeq,beq,Aiq,biq,H,f); break;
    case 2: QPSolution = mpc_solver2_.Update_Solver(Aeq,beq,Aiq,biq,H,f); break;
    case 3: QPSolution = mpc_solver3_.Update_Solver(Aeq,beq,Aiq,biq,H,f); break;
    default: ;
  }  

  vector<double> sol_stdvec(QPSolution.data(),QPSolution.data() + Vars_Num_);
  sol_ = sol_stdvec;
  return QPSolution;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "listener");
  Digit_MPC digit_mpc;

  ros::NodeHandle n;
  ros::Rate loop_rate(200);
  ros::Publisher mpc_res_pub = n.advertise<Digit_Ros::mpc_info>("mpc_res", 1000);
  int count = 0;

  VectorXd QPSolution = VectorXd::Zero(digit_mpc.get_Var_Num(),1);
  VectorXd cmd_pel_pos = VectorXd::Zero(3,1);
  VectorXd cmd_pel_vel = VectorXd::Zero(3,1); 
  VectorXd cmd_left_foot = VectorXd::Zero(3,1);
  VectorXd cmd_right_foot = VectorXd::Zero(3,1);
  VectorXd foot_change = VectorXd::Zero(2,1);

  while (ros::ok()){
    double traj_time = 0;
    int stance_leg = 1;
    auto mpc_time_start = std::chrono::system_clock::now();

    // solve for results
    traj_time = digit_mpc.get_traj_time();
    if(traj_time >= digit_mpc.get_dstime()/2 && traj_time < digit_mpc.get_steptime() - digit_mpc.get_dstime()/2){
      int mpc_index = floor((traj_time - digit_mpc.get_dstime()/2) / 0.1);
      if(traj_time - digit_mpc.get_dstime()/2 - mpc_index * 0.1 < 0.1){
        VectorXd mpc_pel_pos = digit_mpc.get_pel_pos();
        VectorXd mpc_pel_vel = digit_mpc.get_pel_vel();
        VectorXd mpc_pel_ref = digit_mpc.get_pel_ref();
        VectorXd mpc_f_init = digit_mpc.get_foot_pos();
        VectorXd mpc_obs_info = digit_mpc.get_obs_info();

        std::vector<double> q_init = {mpc_pel_pos(0), mpc_pel_vel(0), mpc_pel_pos(1), mpc_pel_vel(1)};
        std::vector<double> q_ref(mpc_pel_ref.data(), mpc_pel_ref.data() + mpc_pel_ref.size());
        std::vector<double> f_init(mpc_f_init.data(), mpc_f_init.data() + mpc_f_init.size());
        std::vector<double> f_param = {0, 0, mpc_pel_ref(1) * (digit_mpc.get_steptime() - digit_mpc.get_dstime()), 0, 0, digit_mpc.get_foot_wdith()};
        std::vector<double> qo_ic(mpc_obs_info.data(), mpc_obs_info.data() + 2);
        std::vector<double> qo_tan(mpc_obs_info.data() + 2, mpc_obs_info.data() + mpc_obs_info.size());
        std::vector<double> rt = {0.1 - (traj_time - digit_mpc.get_dstime()/2 - mpc_index * 0.1)};
        if(mpc_index > 2)
          rt[0] = 0.1 - digit_mpc.get_dstime()/2  - (traj_time - digit_mpc.get_dstime()/2 - mpc_index * 0.1);

        vector<vector<double>> mpc_input;
        mpc_input.push_back(q_init);
        mpc_input.push_back(q_ref);
        mpc_input.push_back(f_init);
        mpc_input.push_back(f_param);
        mpc_input.push_back(qo_ic);
        mpc_input.push_back(qo_tan);
        mpc_input.push_back(rt);

        QPSolution = digit_mpc.Update_MPC_(mpc_index,mpc_input);
        auto mpc_time = duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - mpc_time_start);
        //cout << "solving time: " << mpc_time.count() << endl;
      }
      if(digit_mpc.get_stance_leg() == 1){
        foot_change << QPSolution(51), QPSolution(106);
/*         cout << "this is a left step, right foot on ground" << endl;
        cout << "initial step:" << endl << digit_mpc.get_foot_pos() << endl;
        cout << "goal step:" << endl << digit_mpc.get_foot_pos() + foot_change << endl; */
      }
      else{
        foot_change << QPSolution(51), QPSolution(106);
/*         cout << "this is a right step, left foot on ground" << endl;
        cout << "initial step:" << endl << digit_mpc.get_foot_pos() << endl;
        cout << "goal step:" << endl << digit_mpc.get_foot_pos() + foot_change << endl; */
      }
      double dt = (traj_time - digit_mpc.get_dstime()/2) / (digit_mpc.get_steptime() - digit_mpc.get_dstime()/2);
      dt = 1;
      cmd_pel_pos << (1 - dt) * QPSolution(0) + dt * QPSolution(2), (1 - dt) * QPSolution(55) + dt * QPSolution(57), 1;
      cmd_pel_vel << (1 - dt) * QPSolution(1) + dt * QPSolution(3), (1 - dt) * QPSolution(56) + dt * QPSolution(58), 0;
      cmd_left_foot.block(0,0,2,1) = digit_mpc.get_foot_pos() + foot_change;
      cmd_right_foot.block(0,0,2,1) = digit_mpc.get_foot_pos() + foot_change;
    }
    
    // interpolates result
    Digit_Ros::mpc_info msg;
    std::copy(cmd_pel_pos.data(),cmd_pel_pos.data() + 3,msg.pel_pos_cmd.begin());
    std::copy(cmd_pel_vel.data(),cmd_pel_vel.data() + 3,msg.pel_vel_cmd.begin());
    std::copy(cmd_left_foot.data(),cmd_left_foot.data() + 3,msg.foot_left_cmd.begin());
    std::copy(cmd_right_foot.data(),cmd_right_foot.data() + 3,msg.foot_right_cmd.begin());
    mpc_res_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  //ros::Subscriber sub = n.subscribe("/digit_state", 100, chatterCallback);
  ros::spin();
  return 0;
  //ros::spin();
}