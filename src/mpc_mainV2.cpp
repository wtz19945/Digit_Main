#include "mpc_mainV2.hpp"
#include "utilities.hpp"
#include "input_listener.hpp"
#include "rosbag/bag.h"

using namespace Eigen;
namespace fs = std::filesystem;
using namespace std::chrono;

Digit_MPC::Digit_MPC(bool run_sim)
{
  mpc_sub_ = node_handler_.subscribe("/digit_state", 10, &Digit_MPC::MPCInputCallback, this);
  pel_pos_ = VectorXd::Zero(3,1);
  pel_vel_ = VectorXd::Zero(3,1);
  theta_ = VectorXd::Zero(3,1);
  dtheta_ = VectorXd::Zero(3,1);
  pel_ref_ = VectorXd::Zero(4,1);
  foot_pos_ = VectorXd::Zero(2,1);
  obs_info_ = VectorXd::Zero(7,1);
  pel_pos_actual_ = VectorXd::Zero(3,1);
  right_swing_ = VectorXd::Zero(3,1);
  left_swing_ = VectorXd::Zero(3,1);
  traj_time_ = 0;
  stance_leg_ = 1;

  // Casadi MPC QP Matrix evaluation function
  std::string prefix_lib = fs::current_path().parent_path().string();
  left_step0_matrix_ = casadi::external("LeftStart_Step0V3", prefix_lib + "/catkin_ws/src/Digit_Main/mpc_lib_stable/LeftStartQP_Step0_Foot_MIQP.so");
  left_step1_matrix_ = casadi::external("LeftStart_Step1V3", prefix_lib + "/catkin_ws/src/Digit_Main/mpc_lib_stable/LeftStartQP_Step1_Foot_MIQP.so");
  left_step2_matrix_ = casadi::external("LeftStart_Step2V3", prefix_lib + "/catkin_ws/src/Digit_Main/mpc_lib_stable/LeftStartQP_Step2_Foot_MIQP.so");
  left_step3_matrix_ = casadi::external("LeftStart_Step3V3", prefix_lib + "/catkin_ws/src/Digit_Main/mpc_lib_stable/LeftStartQP_Step3_Foot_MIQP.so");

  right_step0_matrix_ = casadi::external("RightStart_Step0V3", prefix_lib + "/catkin_ws/src/Digit_Main/mpc_lib_stable/RightStartQP_Step0_Foot_MIQP.so");
  right_step1_matrix_ = casadi::external("RightStart_Step1V3", prefix_lib + "/catkin_ws/src/Digit_Main/mpc_lib_stable/RightStartQP_Step1_Foot_MIQP.so");
  right_step2_matrix_ = casadi::external("RightStart_Step2V3", prefix_lib + "/catkin_ws/src/Digit_Main/mpc_lib_stable/RightStartQP_Step2_Foot_MIQP.so");
  right_step3_matrix_ = casadi::external("RightStart_Step3V3", prefix_lib + "/catkin_ws/src/Digit_Main/mpc_lib_stable/RightStartQP_Step3_Foot_MIQP.so");

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
  std::shared_ptr<cpptoml::table> config_osc = cpptoml::parse_file(package_path + "/src/config_file/oscmpc_robot_config.toml");

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
  du_cost_ = config->get_qualified_as<double>("MPC-Params.W_du").value_or(0);
  double r1 = config->get_qualified_as<double>("MPC-Params.obs_rad1").value_or(0);
  double r2 = config->get_qualified_as<double>("MPC-Params.obs_rad2").value_or(0);
  mpc_rate_ = config->get_qualified_as<double>("MPC-Params.mpc_rate").value_or(0);
  if(run_sim){
    ux_off_ = config->get_qualified_as<double>("MPC-Params.ux_off_sim").value_or(0);
    uy_off_ = config->get_qualified_as<double>("MPC-Params.uy_off_sim").value_or(0);
  }
  else{
    ux_off_ = config->get_qualified_as<double>("MPC-Params.ux_off").value_or(0);
    uy_off_ = config->get_qualified_as<double>("MPC-Params.uy_off").value_or(0);
  }

  double swf_Qx = config->get_qualified_as<double>("Foot-Params.swf_Qx").value_or(0);
  double swf_Qy = config->get_qualified_as<double>("Foot-Params.swf_Qy").value_or(0);
  double swf_Qz = config->get_qualified_as<double>("Foot-Params.swf_Qz").value_or(0);
  double swf_xy_r1 = config->get_qualified_as<double>("Foot-Params.swf_xy_r1").value_or(0);
  double swf_xy_r2 = config->get_qualified_as<double>("Foot-Params.swf_xy_r2").value_or(0);
  double swf_z_r1 = config->get_qualified_as<double>("Foot-Params.swf_z_r1").value_or(0);
  double swf_z_r2 = config->get_qualified_as<double>("Foot-Params.swf_z_r2").value_or(0);
  double swf_obs_Qxy = config->get_qualified_as<double>("Foot-Params.swf_obs_Qxy").value_or(0);
  double swf_obs_Qz = config->get_qualified_as<double>("Foot-Params.swf_obs_Qz").value_or(0);
  swf_z_frac_ = config->get_qualified_as<double>("Foot-Params.swf_z_frac").value_or(0);
  step_height_ = config_osc->get_qualified_as<double>("Walk-Params.step_height").value_or(0);
  step_z_max_ = config->get_qualified_as<double>("Foot-Params.z_max").value_or(0);
  double M = config->get_qualified_as<double>("Foot-Params.M").value_or(0);

  step_time_ = config_osc->get_qualified_as<double>("Walk-Params.step_time").value_or(0);
  ds_time_ = config_osc->get_qualified_as<double>("Walk-Params.ds_time").value_or(0);
  double th = config->get_qualified_as<double>("Foot-Params.th").value_or(0);
  assert(th <= 45 && th >= -45);

  f_length_ = {flx, fly};
  Weights_ss_ = {Wx, Wdx, Wy, Wdy, Wux, Wuy, Wdux, Wduy, Wobs};
  Weights_ds_ = {0, 2500, 0, 2500, 10000, 10000, 100, 6000, 15000};
  Weights_swf_Q_ = {swf_Qx, swf_Qy, swf_Qz};
  Weights_swf_param_ = {swf_xy_r1, swf_xy_r2, swf_z_r1, swf_z_r2, swf_obs_Qxy, swf_obs_Qz, swf_z_frac_,step_height_,step_z_max_, M, th * M_PI/180};

  r_ = {r1, r2};
  f_width_ = config->get_qualified_as<double>("MPC-Params.foot_width").value_or(0);
  height_ = config->get_qualified_as<double>("MPC-Params.height").value_or(0);
  // QP variable
  NPred_ = 4;
  Nodes_ = NPred_ * 4 + 1;
  nx_ = 2;
  
  // initialize solvers
  if(NPred_ == 4){
    Cons_Num_ = {296,292,287,283};
    Vars_Num_ = 181;
  }
  else if(NPred_ == 5){
    Cons_Num_ = {318,316,313,311};
    Vars_Num_ = 201;
  }
  else{
    Cons_Num_ = {373,371,368,366};
    Vars_Num_ = 236;
  }

  for(int i = 0; i<Vars_Num_;i++){
    sol_.push_back(0);
    sol_init_.push_back(0);
  }
  //
  mpc_solver0_ = MPC_Solver(Cons_Num_[0],Vars_Num_, NPred_);
  mpc_solver1_ = MPC_Solver(Cons_Num_[1],Vars_Num_, NPred_);
  mpc_solver2_ = MPC_Solver(Cons_Num_[2],Vars_Num_, NPred_);
  mpc_solver3_ = MPC_Solver(Cons_Num_[3],Vars_Num_, NPred_);
}

VectorXd Digit_MPC::linspace(double start, double end, int num){
    Eigen::VectorXd result(num,1);
    if (num == 1) {
        result[0] = start;
    } else {
        double step = (end - start) / (num - 1);
        for (int i = 0; i < num; ++i) {
            result[i] = start + step * i;
        }
    }
    return result;
}

// get mpc inputs (robot foot&base states)
void Digit_MPC::MPCInputCallback(const Digit_Ros::digit_state& msg) {
  std::copy(msg.pel_pos.begin(), msg.pel_pos.begin() + 3, pel_pos_.data());
  std::copy(msg.pel_vel.begin(), msg.pel_vel.begin() + 3, pel_vel_.data());
  std::copy(msg.pel_rot.begin(), msg.pel_rot.begin() + 3, theta_.data());
  std::copy(msg.pel_omg.begin(), msg.pel_omg.begin() + 3, dtheta_.data());
  std::copy(msg.pel_ref.begin(), msg.pel_ref.begin() + 4, pel_ref_.data());
  std::copy(msg.foot_pos.begin(), msg.foot_pos.begin() + 2, foot_pos_.data());
  std::copy(msg.obs_info.begin(), msg.obs_info.begin() + 7, obs_info_.data());
  std::copy(msg.left_toe_pos.begin(), msg.left_toe_pos.begin() + 3, left_swing_.data());
  std::copy(msg.right_toe_pos.begin(), msg.right_toe_pos.begin() + 3, right_swing_.data());
  std::copy(msg.pel_pos_actual.begin(), msg.pel_pos_actual.begin() + 3, pel_pos_actual_.data());

  traj_time_ = msg.traj_time;
  stance_leg_ = msg.stance_leg;
}

// Solve MPC to get new task space command
VectorXd Digit_MPC::Update_MPC_(int traj_time, std::vector<std::vector<double>> mpc_input){
  VectorXd QPSolution;
  int mpc_in = traj_time;
  std::vector<double> input;
  input.insert(input.end(), mpc_input[0].begin(), mpc_input[0].end());
  input.insert(input.end(), mpc_input[1].begin(), mpc_input[1].end());
  input.insert(input.end(), mpc_input[2].begin(), mpc_input[2].end());
  input.insert(input.end(), f_length_.begin(), f_length_.end());
  input.insert(input.end(), mpc_input[3].begin(), mpc_input[3].end());
  input.insert(input.end(), mpc_input[4].begin(), mpc_input[4].end());
  input.insert(input.end(), Weights_ss_.begin(), Weights_ss_.end());
  input.insert(input.end(), r_.begin(), r_.end());
  input.insert(input.end(), mpc_input[5].begin(), mpc_input[5].end());
  input.insert(input.end(), mpc_input[6].begin(), mpc_input[6].end());
  input.insert(input.end(), mpc_input[7].begin(), mpc_input[7].end());
  input.insert(input.end(), mpc_input[8].begin(), mpc_input[8].end());
  input.insert(input.end(), mpc_input[9].begin(), mpc_input[9].end());
  input.insert(input.end(), mpc_input[10].begin(), mpc_input[10].end());
  input.insert(input.end(), mpc_input[11].begin(), mpc_input[11].end());
  input.insert(input.end(), Weights_swf_Q_.begin(), Weights_swf_Q_.end());
  input.insert(input.end(), mpc_input[12].begin(), mpc_input[12].end());
  input.insert(input.end(), Weights_swf_param_.begin(), Weights_swf_param_.end());
  input.insert(input.end(), mpc_input[13].begin(), mpc_input[13].end());
  std::vector<casadi::DM> MPC_arg = {input,sol_init_};  
  std::vector<casadi::DM> res;

  if(stance_leg_ == -1){
      switch(mpc_in){
        case 0: res = left_step0_matrix_(MPC_arg); break;
        case 1: res = left_step1_matrix_(MPC_arg); break;
        case 2: res = left_step2_matrix_(MPC_arg); break;
        case 3: res = left_step3_matrix_(MPC_arg); break;
        case 4: res = left_step3_matrix_(MPC_arg); break;
        default: ;
      }
  }
  else{
      switch(mpc_in){
        case 0: res = right_step0_matrix_(MPC_arg); break;
        case 1: res = right_step1_matrix_(MPC_arg); break;
        case 2: res = right_step2_matrix_(MPC_arg); break;
        case 3: res = right_step3_matrix_(MPC_arg); break;
        case 4: res = right_step3_matrix_(MPC_arg); break;
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
    case 4: QPSolution = mpc_solver3_.Update_Solver(Aeq,beq,Aiq,biq,H,f); break;
    default: ;
  }  

  //vector<double> sol_stdvec(QPSolution.data(),QPSolution.data() + Vars_Num_);
  //sol_ = sol_stdvec;
  return QPSolution;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "listener");
  
  ros::NodeHandle n;
  bool run_sim = true;
  n.getParam("sim_mode",run_sim);
  Digit_MPC digit_mpc(run_sim);
  ros::Rate loop_rate(digit_mpc.get_mpcrate());
  ros::Publisher mpc_res_pub = n.advertise<Digit_Ros::mpc_info>("mpc_res", 10);

  // Use rosbad logging
  rosbag::Bag bag;
  std::time_t now = std::time(nullptr);
  std::stringstream date;
  date << std::put_time(std::localtime(&now), "%Y_%m_%d_%H_%M_%S");

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
  std::shared_ptr<cpptoml::table> config_osc = cpptoml::parse_file(package_path + "/src/config_file/oscmpc_robot_config.toml");
  int recording = config_osc->get_qualified_as<double>("Other.recording").value_or(0);

  if(recording){
    if(run_sim)
      bag.open(package_path + "/data/Sim_Data/mpc_sim_test_" + date.str() + ".bag", rosbag::bagmode::Write);
    else
      bag.open(package_path + "/data/Hard_Data/mpc_test_" + date.str() + ".bag", rosbag::bagmode::Write);
  }

  int count = 0;

  // MPC ros output
  VectorXd QPSolution = VectorXd::Zero(digit_mpc.get_Var_Num(),1);
  VectorXd cmd_pel_pos = VectorXd::Zero(4,1);
  VectorXd cmd_pel_vel = VectorXd::Zero(4,1); 
  VectorXd cmd_left_foot = VectorXd::Zero(3,1);
  VectorXd cmd_right_foot = VectorXd::Zero(3,1);
  VectorXd swing_foot_cmd = VectorXd::Zero(15,1);
  VectorXd mpc_swf_init = VectorXd::Zero(3,1);

  // MPC info
  VectorXd swing_foot_start = VectorXd::Zero(3,1);
  VectorXd swf_x_ref = VectorXd::Zero(5,1);
  VectorXd swf_y_ref = VectorXd::Zero(5,1);
  VectorXd swf_z_ref = VectorXd::Zero(5,1);
  VectorXd swf_ref = VectorXd::Zero(15,1);
  swf_z_ref << 0.0, 0.1, 0.2, 0.1, 0.0;
  VectorXd foot_change = VectorXd::Zero(2,1);

  // 
  // LIP params like gravity
  double g = 9.81;
  double h = digit_mpc.get_height();
  double w = sqrt(g/h);

  // Keyboard Listener
  int key_mode = -1;
  InputListener input_listener(&key_mode);

  double foot_x_offset = 0;
  double foot_y_offset = 0;
  double dx_offset = 0;
  int Npred = digit_mpc.get_Npred();
  int Nodes = digit_mpc.get_Nodes();
  int counter = 0;
  int stance_leg_prev = 1;
  
  while (ros::ok()){
    double traj_time = 0;
    auto mpc_time_start = std::chrono::system_clock::now();
    // tune offset parameters to account for model mismatch and avoid drifting
    switch(key_mode) {
      case 9:
        foot_x_offset += 0.0001;
        break;
      case 10:
        foot_x_offset -= 0.0001;
        break;
      case 11:
        foot_y_offset += 0.00001;
        break;
      case 12:
        foot_y_offset -= 0.00001;
        break;
    }

    counter++;
    if(counter == 10){
      if(key_mode >=9 && key_mode <=12){
        std::cout << "current offset are" << std::endl;
        std::cout << "x direction   " << foot_x_offset << std::endl;
        std::cout << "y direction   " << foot_y_offset << std::endl;
      }
      counter = 0;
    }
    // solve for results
    traj_time = digit_mpc.get_traj_time();
    if(traj_time >= digit_mpc.get_dstime()/2 && traj_time < digit_mpc.get_steptime() - digit_mpc.get_dstime()/2){
        int mpc_index = floor((traj_time - digit_mpc.get_dstime()/2) / 0.1);
        VectorXd mpc_pel_pos = digit_mpc.get_pel_pos();
        VectorXd mpc_pel_vel = digit_mpc.get_pel_vel();
        VectorXd mpc_pel_ref = digit_mpc.get_pel_ref();
        VectorXd mpc_f_init = digit_mpc.get_foot_pos();
        VectorXd mpc_obs_info = digit_mpc.get_obs_info();   
        VectorXd mpc_swf_cur = digit_mpc.get_swing_foot();

        double foot_width = digit_mpc.get_foot_width();
        double dx_des = mpc_pel_ref(1);
        double dy_des = mpc_pel_ref(3);
        double dy_offset = 0;
        double T = digit_mpc.get_steptime() - digit_mpc.get_dstime();
        if(digit_mpc.get_stance_leg() == 1)
          dy_offset = w * foot_width * tanh(w * T / 2);
        else
          dy_offset = -w * foot_width* tanh(w * T / 2);
        double fx_offset = dx_des / w * ((2 - exp(w*T) - exp(-w*T)) / (exp(w * T) - exp(-w * T)));

        if(stance_leg_prev != digit_mpc.get_stance_leg()){
          //cout << "stance leg change!" << endl;
          swing_foot_start = digit_mpc.get_swing_foot();
          swing_foot_start(0) -= 0.08;
          swf_x_ref << digit_mpc.linspace(swing_foot_start(0), swing_foot_start(0) + 2 * digit_mpc.get_steptime() * dx_des, 5);
          swf_y_ref << digit_mpc.linspace(swing_foot_start(1), swing_foot_start(1) + digit_mpc.get_steptime() * dy_des, 5);
          swf_z_ref << 0.0, digit_mpc.get_z_min() * digit_mpc.get_z_frac(), digit_mpc.get_z_min(), digit_mpc.get_z_min() * digit_mpc.get_z_frac(), 0.0;
        }
        else{
          swf_x_ref << digit_mpc.linspace(swing_foot_start(0), mpc_f_init(0) + foot_change(0), 5);
          swf_y_ref << digit_mpc.linspace(swing_foot_start(1), mpc_f_init(1) + foot_change(1), 5);
          for(int i = 0; i < mpc_index; i++){
              swf_z_ref(i) = swing_foot_cmd(10 + i);
          }
        }

        double drift = 0.0;
        if(swf_z_ref(2) < 0.2)
            drift = (swf_z_ref(2) - 0.2) * 0.2;
        else
            drift = (swf_z_ref(2) - 0.2) * 0.5;
        swf_ref << swf_x_ref, swf_y_ref, swf_z_ref;
        stance_leg_prev = digit_mpc.get_stance_leg();

        std::vector<double> q_init = {mpc_pel_pos(0), mpc_pel_vel(0), mpc_pel_pos(1), mpc_pel_vel(1)};
        std::vector<double> x_ref = {0, dx_des, dx_des, dx_des, dx_des};
        std::vector<double> y_ref = {0, dy_offset + dy_des, -dy_offset + dy_des, dy_offset + dy_des, -dy_offset + dy_des};
        for(int i = 0; i < Npred - 4; i++){
          x_ref.push_back(dx_des);
          y_ref.push_back(std::pow(-1, i) * dy_offset + dy_des);
        }
        std::vector<double> f_init(mpc_f_init.data(), mpc_f_init.data() + mpc_f_init.size());
        std::vector<double> f_param = {0, 0, fx_offset, 0, 0, foot_width};
        std::vector<double> qo_ic(mpc_obs_info.data(), mpc_obs_info.data() + 2);
        std::vector<double> qo_tan(mpc_obs_info.data() + 2, mpc_obs_info.data() + 4);
        std::vector<double> rt = {std::max(0.1 - (traj_time - digit_mpc.get_dstime()/2 - mpc_index * 0.1),0.0)};
        if(mpc_index > 2)
          rt[0] = std::max(0.1 - digit_mpc.get_dstime()/2  - (traj_time - digit_mpc.get_dstime()/2 - mpc_index * 0.1), 0.0);
        std::vector<double> foff = {digit_mpc.get_uxoff() + foot_x_offset, digit_mpc.get_uyoff() + foot_y_offset};
        std::vector<double> du_reff = {h, 0.0};
        if(traj_time - digit_mpc.get_dstime()/2 - mpc_index * 0.1 < 0.03){
          mpc_swf_init = mpc_swf_cur;
          mpc_swf_init(0) -= 0.08;
          mpc_swf_init(2) = std::max(mpc_swf_cur(2) - swing_foot_start(2),0.02);
        }

        std::vector<double> swf_cq(mpc_swf_init.data(), mpc_swf_init.data() + mpc_swf_init.size()); // starting position        
        std::vector<double> swf_rq(swf_ref.data(), swf_ref.data() + swf_ref.size()); // reference traj
        std::vector<double> swf_obs(mpc_obs_info.data() + 4, mpc_obs_info.data() + 7); // foot obs position
        std::vector<double> avd_param{mpc_pel_ref(1) * digit_mpc.get_steptime() * 4 + dx_offset, 8000, 1, 1};
        if(mpc_pel_ref(1) < 0)
          avd_param[2] = -1;

        std::vector<std::vector<double>> mpc_input;
        mpc_input.push_back(q_init);
        mpc_input.push_back(x_ref);
        mpc_input.push_back(y_ref);
        mpc_input.push_back(f_init);
        mpc_input.push_back(f_param);
        mpc_input.push_back(qo_ic);
        mpc_input.push_back(qo_tan);
        mpc_input.push_back(rt);
        mpc_input.push_back(foff);
        mpc_input.push_back(du_reff);
        mpc_input.push_back(swf_cq);
        mpc_input.push_back(swf_rq);
        mpc_input.push_back(swf_obs);
        mpc_input.push_back(avd_param);


        QPSolution = digit_mpc.Update_MPC_(mpc_index,mpc_input);
        auto mpc_time = duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - mpc_time_start);
        //cout << "solving time: " << mpc_time.count() << endl;
      
      if(digit_mpc.get_stance_leg() == 1){
        foot_change << QPSolution(3 * Nodes), QPSolution(6*Nodes + Npred);
      }
      else{
        foot_change << QPSolution(3 * Nodes), QPSolution(6*Nodes + Npred);
      }

      double error = abs(QPSolution(8) - QPSolution(0)) - mpc_pel_ref(1) * digit_mpc.get_steptime() + 0.005;
      if(error < 0 && mpc_pel_ref(1) != 0){
          std::cout << "hhhh acc: " << dx_offset << std::endl;
          dx_offset = std::min(dx_offset + abs(error), 0.6);
      }
      else{
          dx_offset = std::max(dx_offset - 0.01, 0.0);
      }

      int off = 0;
      cmd_pel_pos << QPSolution(0), QPSolution(2 + off), QPSolution(3*Nodes + Npred), QPSolution(3*Nodes + Npred + 2 + off);
      cmd_pel_vel << QPSolution(1), QPSolution(3 + off), QPSolution(3*Nodes + Npred + 1), QPSolution(3*Nodes + Npred + 3 + off);
      cmd_left_foot.block(0,0,2,1) = digit_mpc.get_foot_pos() + foot_change;
      cmd_right_foot.block(0,0,2,1) = digit_mpc.get_foot_pos() + foot_change;
      swing_foot_cmd = QPSolution.block(7*Nodes + 2 * Npred,0,15,1);
    }

    // interpolates result
    Digit_Ros::mpc_info msg;
    std::copy(cmd_pel_pos.data(),cmd_pel_pos.data() + 4,msg.pel_pos_cmd.begin());
    std::copy(cmd_pel_vel.data(),cmd_pel_vel.data() + 4,msg.pel_vel_cmd.begin());
    std::copy(cmd_left_foot.data(),cmd_left_foot.data() + 3,msg.foot_left_cmd.begin());
    std::copy(cmd_right_foot.data(),cmd_right_foot.data() + 3,msg.foot_right_cmd.begin());
    std::copy(swing_foot_cmd.data(),swing_foot_cmd.data() + swing_foot_cmd.size(),msg.swing_foot_cmd.begin());
    std::copy(mpc_swf_init.data(), mpc_swf_init.data() + mpc_swf_init.size(), msg.mpc_swf_cur.begin());
    std::copy(QPSolution.data(), QPSolution.data() + QPSolution.size(), msg.mpc_solution.begin());
    VectorXd f_init = digit_mpc.get_foot_pos();
    VectorXd actual_pos = digit_mpc.get_actual_pel_pos();
    VectorXd mpc_obs_info = digit_mpc.get_obs_info();

    std::copy(f_init.data(), f_init.data() + f_init.size(), msg.f_init.begin());
    std::copy(actual_pos.data(), actual_pos.data() + actual_pos.size(), msg.mpc_pel_pos.begin());
    std::copy(mpc_obs_info.data(), mpc_obs_info.data() + mpc_obs_info.size(), msg.obs_info.begin());
    msg.dx_offset = dx_offset;
    mpc_res_pub.publish(msg);

    if(recording)
      bag.write("mpc_info", ros::Time::now(), msg);
    //cout << "Desired yaw angle is: " << msg.yaw << endl;
    // Check if llapi has become disconnected
    if(ros::isShuttingDown()){
      if(recording)
        bag.close();
      break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  //ros::Subscriber sub = n.subscribe("/digit_state", 100, chatterCallback);
  ros::spin();
  return 0;
  //ros::spin();
}