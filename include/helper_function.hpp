#ifndef HELPER_FUNCTION_H
#define HELPER_FUNCTION_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <cmath>
#include <csignal>
#include <memory>
#include <Eigen/Dense>
#include "cpptoml/include/cpptoml.h"
#include "kin_left_arm.hpp"
#include "kin_right_arm.hpp"
#include "utilities.hpp"

using namespace Eigen;

// Implementation of inverse kinematics for Digit arm.
class IKArmControl {
public:
    IKArmControl(double step_time, double arm_z_int);
    VectorXd update_right_arm(int stepping, int stance_leg, double traj_time, VectorXd wb_q);
    VectorXd update_left_arm(int stepping, int stance_leg, double traj_time, VectorXd wb_q);

private:
    double step_time_;
    double arm_z_int_;
    VectorXd ql_last_;
    VectorXd qr_last_;
};

// Finite state machine for Digit
class StateMachine {
    public:
        StateMachine(double step_time, double ds_time);
        VectorXd get_contact_traj() {return contact_;};
        double get_traj_time() {return traj_time_;};
        int get_stepping_phase() {return stepping_phase_;};
        int get_stance_leg() {return stance_leg_;};
        void update(int cmd, double dt);
    private:
        int stepping_phase_;
        int stance_leg_;
        double traj_time_;
        double step_time_;
        double ds_time_;
        VectorXd contact_;
};

// An obstacle path generator to test the avoidance algorithm when on board obstacle estimiation is not available.
class ObstacleGenerator {
    public:
        ObstacleGenerator();
        int get_avoidance_mode(double key_cmd, int stepping, VectorXd &obs_pos);
    private:
        int avd_mode_;
        int cmd_active_;
        VectorXd obs_pos_;
};

void wrap_theta(double &theta);

VectorXd generate_traj(const VectorXd &x, double t);

#endif //HELPER_FUNCTION_H

