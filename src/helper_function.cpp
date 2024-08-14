#include "helper_function.hpp"

IKArmControl::IKArmControl(double step_time, double arm_z_int)
{
    step_time_ = step_time;
    arm_z_int_ = arm_z_int;
    ql_last_ = VectorXd::Zero(10, 1);
    qr_last_ = VectorXd::Zero(10, 1);
};

VectorXd IKArmControl::update_left_arm(int stepping, int stance_leg, double traj_time, VectorXd wb_q)
{
    VectorXd ql = VectorXd::Zero(10, 1);
    VectorXd p_lh = VectorXd::Zero(3, 1);
    MatrixXd J_lh = MatrixXd::Zero(3, 10);
    VectorXd p_lh_ref = VectorXd::Zero(3, 1);
    VectorXd p_lh_err = VectorXd::Zero(3, 1);

    VectorXd goal_pose = VectorXd::Zero(4, 1);

    // The IK is always in the base frame
    if (stepping == 2)
    {
        if (stance_leg == 1)
        {
            p_lh_ref << 0.02 + 0.1 * sin(M_PI * traj_time / step_time_), +0.25, +arm_z_int_;
        }
        else
        {
            p_lh_ref << 0.02 - 0.1 * sin(M_PI * traj_time / step_time_), +0.25, +arm_z_int_;
        }
    }
    else
    {
        p_lh_ref << +0.02, +0.25, +arm_z_int_;
    }

    // initialize q with current pose
    ql.block(6, 0, 4, 1) = wb_q.block(20, 0, 4, 1);
    kin_left_arm(ql.data(), p_lh.data(), J_lh.data());
    J_lh.block(0, 0, 3, 7) = MatrixXd::Zero(3, 7); // base is fixed for arm IK

    // left arm IK
    double error;
    error = (p_lh - p_lh_ref).norm();
    double iter = 0;

    // IK is warmed started with last solution is goal is close enough. Otherwise initialize with current configuration
    if (error > 0.01)
    {
        while (error > 0.01 && iter < 2)
        {
            // solve for new joint
            ql += J_lh.colPivHouseholderQr().solve(p_lh_ref - p_lh);
            // Clip joints
            ql(6) = std::max(std::min(ql(6), deg2rad(75)), deg2rad(-75));
            ql(7) = std::max(std::min(ql(7), deg2rad(145)), deg2rad(-145));
            ql(8) = std::max(std::min(ql(8), deg2rad(100)), deg2rad(-100));
            ql(9) = std::max(std::min(ql(9), deg2rad(77.5)), deg2rad(-77.5));
            // Evaluate new pos
            kin_left_arm(ql.data(), p_lh.data(), J_lh.data());
            J_lh.block(0, 0, 3, 7) = MatrixXd::Zero(3, 7); // base is fixed for arm IK
            error = (p_lh - p_lh_ref).norm();
            iter++;
        }

        ql_last_ = ql;
    }
    else
    {
        ql = ql_last_;
    }
    goal_pose(0) = ql(6);
    goal_pose(1) = ql(7);
    goal_pose(2) = ql(8);
    goal_pose(3) = ql(9);
    return goal_pose;
};

VectorXd IKArmControl::update_right_arm(int stepping, int stance_leg, double traj_time, VectorXd wb_q)
{
    VectorXd qr = VectorXd::Zero(10, 1);
    VectorXd p_rh = VectorXd::Zero(3, 1);
    MatrixXd J_rh = MatrixXd::Zero(3, 10);
    VectorXd p_rh_ref = VectorXd::Zero(3, 1);
    VectorXd p_rh_err = VectorXd::Zero(3, 1);

    VectorXd goal_pose = VectorXd::Zero(4, 1);
    // The IK is always in the base frame
    if (stepping == 2)
    {
        if (stance_leg == 1)
        {
            p_rh_ref << 0.02 - 0.1 * sin(M_PI * traj_time / step_time_), -0.25, +arm_z_int_;
        }
        else
        {
            p_rh_ref << 0.02 + 0.1 * sin(M_PI * traj_time / step_time_), -0.25, +arm_z_int_;
        }
    }
    else
    {
        p_rh_ref << +0.02, -0.25, +arm_z_int_;
    }

    qr.block(6, 0, 4, 1) = wb_q.block(24, 0, 4, 1);
    kin_right_arm(qr.data(), p_rh.data(), J_rh.data());
    J_rh.block(0, 0, 3, 7) = MatrixXd::Zero(3, 7); // base is fixed for arm IK

    // right arm IKpel_pos_des
    double error = (p_rh - p_rh_ref).norm();
    int iter = 0;
    if (error > 0.01)
    {
        while (error > 0.01 && iter < 2)
        {
            qr += J_rh.colPivHouseholderQr().solve(p_rh_ref - p_rh);
            // Clip joints
            qr(6) = std::max(std::min(qr(6), deg2rad(75)), deg2rad(-75));
            qr(7) = std::max(std::min(qr(7), deg2rad(145)), deg2rad(-145));
            qr(8) = std::max(std::min(qr(8), deg2rad(100)), deg2rad(-100));
            qr(9) = std::max(std::min(qr(9), deg2rad(77.5)), deg2rad(-77.5));
            // Evaluate new pos
            kin_right_arm(qr.data(), p_rh.data(), J_rh.data());
            J_rh.block(0, 0, 3, 7) = MatrixXd::Zero(3, 7); // base is fixed for arm IK
            error = (p_rh - p_rh_ref).norm();
            iter++;
        }
        qr_last_ = qr;
    }
    else
    {
        qr = qr_last_;
    }
    goal_pose(0) = qr(6);
    goal_pose(1) = qr(7);
    goal_pose(2) = qr(8);
    goal_pose(3) = qr(9);
    return goal_pose;
};

StateMachine::StateMachine(double step_time, double ds_time)
{
    stepping_phase_ = 0;
    traj_time_ = 0.0;
    step_time_ = step_time;
    stance_leg_ = 1;
    ds_time_ = ds_time;
    contact_ = VectorXd::Zero(2, 1);
    contact_ << 1, 1;
}

void StateMachine::update(int cmd, double dt)
{
    int change_state = 0;
    if (cmd == 4 || stepping_phase_ > 0)
    {
        traj_time_ = traj_time_ + dt; // use this when the simulator time is slow than real-time
        // traj_time = traj_time + 1 / qp_rate; // use this when the simualator is close to real-time
        if (traj_time_ > step_time_)
        {
            traj_time_ = 0;
            change_state = 1;
        }
        else
        {
            change_state = 0;
        }
        if (stepping_phase_ < 2 && change_state == 1)
            stepping_phase_++;
    }

    // standing
    if (stepping_phase_ == 0)
    {
        contact_ << 1, 1;
    }
    // transit to stepping
    else if (stepping_phase_ == 1)
    {
        contact_ << 1, 1;
    }
    // stepping phase
    else
    {
        if (change_state)
        {
            stance_leg_ *= -1;
        }
        if (stance_leg_ == 1)
        {
            if (ds_time_ == 0)
            {
                contact_ << 0, 1;
            }
            else
            {
                if (traj_time_ <= ds_time_ / 2)
                    contact_ << 0.5 - traj_time_ * 1 / ds_time_, 0.5 + traj_time_ * 1 / ds_time_;
                else if (traj_time_ >= step_time_ - ds_time_ / 2)
                    contact_ << 0 + 1 / ds_time_ * (traj_time_ - step_time_ + ds_time_ / 2), 1 - 1 / ds_time_ * (traj_time_ - step_time_ + ds_time_ / 2);
                else
                    contact_ << 0, 1;
            }
        }
        else
        {
            if (ds_time_ == 0)
            {
                contact_ << 1, 0;
            }
            else
            {
                if (traj_time_ <= ds_time_ / 2)
                    contact_ << 0.5 + traj_time_ * 1 / ds_time_, 0.5 - traj_time_ * 1 / ds_time_;
                else if (traj_time_ >= step_time_ - ds_time_ / 2)
                    contact_ << 1 - 1 / ds_time_ * (traj_time_ - step_time_ + ds_time_ / 2), 0 + 1 / ds_time_ * (traj_time_ - step_time_ + ds_time_ / 2);
                else
                    contact_ << 1, 0;
            }
        }
    }
}

int ObstacleGenerator::get_avoidance_mode(double key_cmd, int stepping, VectorXd &obs_pos)
{
    if (key_cmd == 13 && stepping == 2)
    {
        // Random obstacle generator
/*         if (cmd_active_ == 0)
        {
            cmd_active_ = 1;
            double angle = dis_(gen_);
            obs_pos_ << 0.2 * cos(angle), 0.2 * sin(angle);
        } */

        // Fixed pattern obstacle generator
        if (cmd_active_ == 0)
        {
            avd_mode_ = (avd_mode_ + 1) % 4;
            switch (avd_mode_)
            {
            case 0:
                std::cout << "avoiding backward" << std::endl;
                obs_pos_ << 0.2, 0.0;
                break;
            case 1:
                std::cout << "avoiding forward" << std::endl;
                obs_pos_ << -0.2, 0.0;
                break;
            case 2:
                std::cout << "avoiding right" << std::endl;
                obs_pos_ << 0.0, 0.2;
                break;
            case 3:
                std::cout << "avoiding left" << std::endl;
                obs_pos_ << 0.0, -0.2;
                break;
            default:
                break;
            }
            cmd_active_++;
        }

        obs_pos = obs_pos_;
        return avd_mode_;
    }
    else
    {
        cmd_active_ = 0;
        obs_pos_ << 5.0, 5.0;
        obs_pos = obs_pos_;
        return -1;
    }
}

void EllipseSolver::update_solver(const double a_x, const double a_y, const VectorXd& pos_obs, const VectorXd& pos_robot, const double phi){
    const double xo = pos_obs(0);
    const double yo = pos_obs(1);
    const double xr = pos_robot(0);
    const double yr = pos_robot(1);

    if(!initialized_){
        // Find initial guess based on relative position
        VectorXd error = pos_robot - pos_obs;
        R_ << cos(phi), -sin(phi), sin(phi), cos(phi);
        error = R_.transpose() * error;
        double x0 = 0;
        if (error(0) >= 0 && error(1) >= 0) {
            x0 = 0.25 * M_PI;
        } else if (error(0) < 0 && error(1) >= 0) {
            x0 = 0.75 * M_PI;
        } else if (error(0) < 0 && error(1) < 0) {
            x0 = -0.75 * M_PI;
        } else {
            x0 = -0.25 * M_PI;
        }
        xn_ = xo + a_x*cos(x0)*cos(phi) - a_y*sin(x0)*sin(phi);
        yn_ = yo + a_x*cos(x0)*sin(phi) + a_y*sin(x0)*cos(phi);
        initialized_ = true;
    }

    // Solve
    ComputeEllipseInfo(xn_, xr, xo, yn_, yr, yo, phi, lambda_, a_x, a_y, f_, J_);
    int iter = 0;
    while(f_.norm() > tor_){
        sol_ = J_.colPivHouseholderQr().solve(f_);
        xn_ -= sol_(0);
        yn_ -= sol_(1);
        lambda_ -= sol_(2);
        ComputeEllipseInfo(xn_, xr, xo, yn_, yr, yo, phi, lambda_, a_x, a_y, f_, J_);
        iter++;
        if(iter > 10) break;
    }
}

// Function Definitions
void EllipseSolver::ComputeEllipseInfo(double xe, double xr, double xc, double ye, double yr,
                        double yc, double t, double lambda, double a, double b,
                        VectorXd& f_Eigen, MatrixXd& Jf_Eigen)
{
  double t10;
  double t11;
  double t2;
  double t20;
  double t21;
  double t23;
  double t25;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  // ComputeEllipseInfo
  //     [F,Jf] = ComputeEllipseInfo(XE,XR,XC,YE,YR,YC,T,LAMBDA,A,B)
  //     This function was generated by the Symbolic Math Toolbox version 24.1.
  //     14-Aug-2024 15:03:53
  t2 = std::cos(t);
  t3 = std::sin(t);
  t6 = 1.0 / (a * a);
  t7 = 1.0 / (b * b);
  t4 = t2 * t2;
  t5 = t3 * t3;
  t10 = -xe + xc;
  t11 = -ye + yc;
  t20 = t2 * t10 + t3 * t11;
  t21 = t2 * t11 - t3 * t10;
  t10 = t2 * t3;
  t23 = lambda * (t10 * t6 * 2.0 - t10 * t7 * 2.0);
  t11 = t2 * t6 * t20 * 2.0;
  t25 = t3 * t6 * t20 * 2.0;
  t2 = t2 * t7 * t21 * 2.0;
  t10 = t3 * t7 * t21 * 2.0;
  f_Eigen[0] = (xe * 2.0 - xr * 2.0) - lambda * (t11 - t10);
  f_Eigen[1] = (ye * 2.0 - yr * 2.0) - lambda * (t25 + t2);
  f_Eigen[2] = (t6 * (t20 * t20) + t7 * (t21 * t21)) - 1.0;
  t11 = -t11 + t10;
  t10 = -t25 - t2;
  Jf_Eigen(0,0) = lambda * (t4 * t6 * 2.0 + t5 * t7 * 2.0) + 2.0;
  Jf_Eigen(0,1) = t23;
  Jf_Eigen(0,2) = t11;
  Jf_Eigen(1,0) = t23;
  Jf_Eigen(1,1) = lambda * (t4 * t7 * 2.0 + t5 * t6 * 2.0) + 2.0;
  Jf_Eigen(1,2) = t10;
  Jf_Eigen(2,0) = t11;
  Jf_Eigen(2,1) = t10;
  Jf_Eigen(2,2) = 0.0;
}

void wrap_theta(double &theta)
{
    if (theta > M_PI)
    {
        theta -= 2 * M_PI;
    }
    else if (theta < -M_PI)
    {
        theta += 2 * M_PI;
    }
    else
    {
        ;
    }
};
