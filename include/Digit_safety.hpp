#ifndef SAFETY_MANAGER_H_
#define SAFETY_MANAGER_H_

#include <unistd.h>
#include <iostream>
#include <cmath>
#include <csignal>
#include <memory>
#include <Eigen/Dense>
#include "cpptoml/include/cpptoml.h"

using namespace Eigen;

double deg2rad(double deg);
class Digit_safety{
    public:
    Digit_safety(){};
    Digit_safety(int wd_sz, int j_sz);

    bool checkSafety();
    void resetSafety();
    void updateSafety(VectorXd cur_q, VectorXd cur_dq);

    private:
    VectorXd jp_max; // joint position limit 
    VectorXd jp_min; // joint position limit 
    VectorXd jv_lim; // joint velocity limit
    bool safety_trigger;
    int unsafe_count;
    int unsafe_limit;
};

#endif //SAFETY_MANAGER_H_

