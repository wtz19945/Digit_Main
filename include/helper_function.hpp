#ifndef HELPER_FUNCTION_H
#define HELPER_FUNCTION_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <cmath>
#include <csignal>
#include <memory>
#include <random>

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
    void update_steptime(const double step_time_new) {step_time_new;};

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
        void update_time_next(const double step_time_next) {step_time_next_ = step_time_next;};
        double get_current_steptime() {return step_time_;};
        
    private:
        int stepping_phase_;
        int stance_leg_;
        double traj_time_;
        double step_time_;
        double step_time_next_;
        double ds_time_;
        VectorXd contact_;
};

// An obstacle path generator to test the avoidance algorithm when on board obstacle estimiation is not available.
class ObstacleGenerator {
    public:
        ObstacleGenerator() : gen_(rd_()), dis_(0, 2 * M_PI), avd_mode_(0), cmd_active_(0), obs_pos_(VectorXd::Zero(2,1)) {};
        int get_avoidance_mode(double key_cmd, int stepping, VectorXd &obs_pos);
    private:
        int avd_mode_;
        int cmd_active_;
        VectorXd obs_pos_;
        std::random_device rd_;
        std::mt19937 gen_; // Mersenne Twister generator
        std::uniform_real_distribution<> dis_; // Uniform distribution
};

// Ellipse Solver: This class finds the closet point on the obstacle represented as an ellipse to the robot 
class EllipseSolver {

    public:
        EllipseSolver() : xn_(0), yn_(0), lambda_(0), tor_(1e-6), initialized_(0), R_(MatrixXd::Zero(2,2)),
                          f_(VectorXd::Zero(3,1)), J_(MatrixXd::Zero(3,3)), sol_(VectorXd::Zero(3,1)) {};
        EllipseSolver(double tor) : xn_(0), yn_(0), lambda_(0), tor_(tor), initialized_(0), R_(MatrixXd::Zero(2,2)),
                          f_(VectorXd::Zero(3,1)), J_(MatrixXd::Zero(3,3)), sol_(VectorXd::Zero(3,1)) {};
                          
        void ComputeEllipseInfo(double xe, double xr, double xc, double ye, double yr,
                                double yc, double t, double lambda, double a, double b,
                                VectorXd& f_Eigen, MatrixXd& Jf_Eigen);

        void update_solver(const double a_x, const double a_y, const VectorXd& pos_obs, const VectorXd& pos_robot, const double phi);
        double get_x() {return xn_;};
        double get_y() {return yn_;};
    private:
        double xn_;        // x coordinate
        double yn_;        // y coordinate
        double lambda_;    // Lagragian in ellipse optim solver
        double tor_;       // Tolerance used in solver
        bool initialized_; // Solver is initialized
        MatrixXd R_;
        VectorXd f_;
        MatrixXd J_;
        VectorXd sol_;
};

void wrap_theta(double &theta);
#endif //HELPER_FUNCTION_H

