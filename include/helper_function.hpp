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



#endif //HELPER_FUNCTION_H

