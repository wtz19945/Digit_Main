#ifndef OSC_CONTROL_H
#define OSC_CONTROL_H

#include <unistd.h>
#include <iostream>
#include <cmath>
#include <csignal>
#include <memory>
#include <Eigen/Dense>
#include "cpptoml/include/cpptoml.h"
#include "OsqpEigen/OsqpEigen.h"
 
using namespace Eigen;

class OSC_Control{
    public:
    OSC_Control(){};
    OSC_Control(std::shared_ptr<cpptoml::table> config);

    void setupQPVector(VectorXd des_acc_pel, VectorXd des_acc, VectorXd des_acc_toe, VectorXd G);

    void setupQPMatrix(MatrixXd Weight_pel, MatrixXd Weight_ToeF, MatrixXd Weight_ToeB, MatrixXd M, 
                       MatrixXd B, MatrixXd Spring_Jaco, MatrixXd left_toe_jaco_fa, 
                       MatrixXd left_toe_back_jaco_fa, MatrixXd right_toe_jaco_fa, MatrixXd right_toe_back_jaco_fa,
                       MatrixXd left_toe_rot_jaco_fa, MatrixXd right_toe_rot_jaco_fa);

    void updateQPVector(VectorXd des_acc_pel, VectorXd des_acc, VectorXd des_acc_toe, VectorXd G);

    void updateQPMatrix(MatrixXd Weight_pel, MatrixXd Weight_ToeF, MatrixXd Weight_ToeB, MatrixXd M,
                        MatrixXd B, MatrixXd Spring_Jaco, MatrixXd left_toe_jaco_fa, 
                        MatrixXd left_toe_back_jaco_fa, MatrixXd right_toe_jaco_fa, MatrixXd right_toe_back_jaco_fa,
                        MatrixXd left_toe_rot_jaco_fa, MatrixXd right_toe_rot_jaco_fa);

    void setUpQP(bool mute_solver);
    void updateQP();
    VectorXd solveQP();

    private:
    // get Coulomb friction constraint matrix
    MatrixXd get_fric_cons(double mu);

    OsqpEigen::Solver solver;
    bool QP_initialized;

    double Wcom;
    double Wff;
    double Wfb;
    double force_max;
    double mu;

    MatrixXd Weight_ToeF;
    MatrixXd Weight_ToeB;
    MatrixXd Weight_pel;

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
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::SparseMatrix<double> hessian;
    std::vector<Eigen::Triplet<double>> coefficients;

    // QP Vector
    Eigen::VectorXd gradient; 
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    Eigen::VectorXd QPSolution;
};

#endif //OSC_CONTROL_H

