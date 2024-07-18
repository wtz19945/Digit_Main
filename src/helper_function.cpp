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

ObstacleGenerator::ObstacleGenerator()
{
    avd_mode_ = 0;
    cmd_active_ = 0;
    obs_pos_ = VectorXd::Zero(2,1);
}

int ObstacleGenerator::get_avoidance_mode(double key_cmd, int stepping, VectorXd &obs_pos)
{
    if (key_cmd == 13 && stepping == 2)
    {
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
                obs_pos_ << -0.14, 0.14;
                break;
            case 2:
                std::cout << "avoiding right" << std::endl;
                obs_pos_ << 0.0, 0.2;
                break;
            case 3:
                std::cout << "avoiding left" << std::endl;
                obs_pos_ << 0.14, -0.14;
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
