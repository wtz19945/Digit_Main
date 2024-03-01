#ifndef OSC_CONTROLV2_H
#define OSC_CONTROLV2_H

#include <unistd.h>
#include <iostream>
#include <cmath>
#include <csignal>
#include <memory>
#include <Eigen/Dense>
#include "cpptoml/include/cpptoml.h"
#include "OsqpEigen/OsqpEigen.h"
#include "osqp.h"

using namespace Eigen;

class MatrixTrip{
  public:
  MatrixTrip(int row_in, int col_in, double value_in) {row = row_in; col = col_in; value = value_in;};
  ~MatrixTrip() {;};
  int row;
  int col;
  double value;
};

class OSC_ControlV2{
    public:
    OSC_ControlV2(){};
    OSC_ControlV2(std::shared_ptr<cpptoml::table> config);
    ~OSC_ControlV2();
    void setupQPVector(VectorXd des_acc_pel, VectorXd des_acc, VectorXd des_acc_toe, VectorXd G, VectorXd contact);

    void setupQPMatrix(MatrixXd Weight_pel, MatrixXd M, 
                       MatrixXd B, MatrixXd Spring_Jaco, MatrixXd left_toe_jaco_fa, 
                       MatrixXd left_toe_back_jaco_fa, MatrixXd right_toe_jaco_fa, MatrixXd right_toe_back_jaco_fa,
                       MatrixXd left_toe_rot_jaco_fa, MatrixXd right_toe_rot_jaco_fa);

    void updateQPVector(VectorXd des_acc_pel, VectorXd des_acc, VectorXd des_acc_toe, VectorXd G, VectorXd contact);

    void updateQPMatrix(MatrixXd Weight_pel, MatrixXd M,
                        MatrixXd B, MatrixXd Spring_Jaco, MatrixXd left_toe_jaco_fa, 
                        MatrixXd left_toe_back_jaco_fa, MatrixXd right_toe_jaco_fa, MatrixXd right_toe_back_jaco_fa,
                        MatrixXd left_toe_rot_jaco_fa, MatrixXd right_toe_rot_jaco_fa, VectorXd contact);

    int setUpQP(bool mute_solver);
    void updateQP();
    VectorXd solveQP(); 

    private:
    // get Coulomb friction constraint matrix
    MatrixXd get_fric_cons(double mu);

    double Wcom,Wcomx,Wcomy,Wcomz,Wcomrz,Wcomry,Wcomrx;
    double Wff;
    double Wfb;
    double Wffsw;
    double Wfbsw;
    double force_max;
    double mu;

    MatrixXd Weight_ToeF;
    MatrixXd Weight_ToeB;
    MatrixXd Weight_ToeFsw;
    MatrixXd Weight_ToeBsw;

    MatrixXd Weight_pel_;
    MatrixXd Weight_pelst;
    VectorXd ddq_limit;
    VectorXd u_limit;
    VectorXd tor_limit;
    VectorXd f_limit_max;
    VectorXd f_limit_min;
    VectorXd f_cons_min;
    VectorXd qt;

    // QP Size
    int Vars_Num;
    int Cons_Num;

    // QP Matrix
    MatrixXd constraint_full;
    MatrixXd hessian_full;

    // QP Vector
    Eigen::VectorXd gradient; 
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    Eigen::VectorXd QPSolution_;

    // osqp solver
    OSQPWorkspace *work;
    OSQPSettings  *settings;
    OSQPData      *data;

    // osqp data
    c_float *losqp;
    c_float *uosqp;
    c_float *qosqp;
    std::vector<MatrixTrip> coefficients_con;
    std::vector<MatrixTrip> coefficients_hes;
};

#endif //OSC_CONTROLV2_H

