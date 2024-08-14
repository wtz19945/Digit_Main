#include "mpc_solver.hpp"
using namespace Eigen;
using namespace std;

MPC_Solver::MPC_Solver(int Cons_Num, int Vars_Num){
    Cons_Num_ = Cons_Num;
    Vars_Num_ = Vars_Num;
    QP_initialized_ = 0;

    hessian_.resize(Vars_Num_,Vars_Num_);
    linearMatrix_.resize(Cons_Num_,Vars_Num_);
    gradient_ = VectorXd::Zero(Vars_Num_,1);
    lowerBound_ = VectorXd::Zero(Cons_Num_,1);
    upperBound_ = VectorXd::Zero(Cons_Num_,1);
    sol_ = VectorXd::Zero(Vars_Num_,1);
}

VectorXd MPC_Solver::Update_Solver(const casadi::DM& Aeq, const casadi::DM& beq,const casadi::DM& Aiq, 
                                    const casadi::DM& biq,const casadi::DM& H, const casadi::DM& f){
    // copy data
    //std::memcpy(gradient_.data(), f.ptr(), sizeof(double)*Vars_Num_);
/*     for(int i = 0; i<Vars_Num_; i++)
        gradient_(i) = (double)f(i);
    std::memcpy(lowerBound_.data(), beq.ptr(), sizeof(double)*Aeq.size1());
    std::memcpy(upperBound_.data(), beq.ptr(), sizeof(double)*Aeq.size1());
    std::memcpy(upperBound_.data() + Aeq.size1(), biq.ptr(), sizeof(double)*Aiq.size1());
    for(int i=0;i<Aiq.size1();i++){
        lowerBound_(i + Aeq.size1()) = -OsqpEigen::INFTY;
    } */

    for(int i = 0; i<Vars_Num_; i++){
        gradient_(i) = (double)f(i);
    }

    for(int i = 0; i < Cons_Num_; i++){
        if(i < Aeq.size1()){
            lowerBound_(i) = (double)beq(i);
            upperBound_(i) = (double)beq(i);
        }
        else{
            lowerBound_(i) = -OsqpEigen::INFTY;
            upperBound_(i) = (double)biq(i - Aeq.size1());
        }
    }
    
    if(QP_initialized_ == 0){
        for(int i=0;i<Vars_Num_;i++){
            for(int j=0;j<Vars_Num_;j++){
                if((double)H(j,i) != 0){
                    hessian_.insert(j,i) = (double)H(j,i);
                    hessian_coeff_.push_back(Eigen::Triplet<double>(j,i,(double)H(j,i))); // create 
                }
            }
        }

        for(int i=0;i<Vars_Num_;i++){
            for(int j=0;j<Aeq.size1() + Aiq.size1();j++){
                if(j< Aeq.size1()){
                    if((double)Aeq(j,i) != 0){
                        linearMatrix_.insert(j,i) = (double)Aeq(j,i);
                        linearm_coeff_.push_back(Eigen::Triplet<double>(j,i,(double)Aeq(j,i)));
                    }
                    }
                    else{
                    if((double)Aiq(j-Aeq.size1(),i) != 0){
                        linearMatrix_.insert(j,i) = (double)Aiq(j-Aeq.size1(),i);
                        linearm_coeff_.push_back(Eigen::Triplet<double>(j,i,(double)Aiq(j-Aeq.size1(),i)));
                    }
                }
            }
        }
        solver_.settings()->setWarmStart(true);
        solver_.data()->setNumberOfVariables(Vars_Num_);
        solver_.data()->setNumberOfConstraints(Cons_Num_);
        solver_.data()->setHessianMatrix(hessian_);
        solver_.data()->setGradient(gradient_);
        solver_.data()->setLinearConstraintsMatrix(linearMatrix_);
        solver_.data()->setLowerBound(lowerBound_);
        solver_.data()->setUpperBound(upperBound_);
        solver_.settings()->setVerbosity(0);
        solver_.initSolver();
        QP_initialized_ = 1;
    }
    else{
        int coeff_iter = 0;
        for (int k=0; k<hessian_.outerSize(); ++k){
            for (SparseMatrix<double>::InnerIterator it(hessian_,k); it; ++it){
                it.valueRef() = (double)H(hessian_coeff_[coeff_iter].row(),hessian_coeff_[coeff_iter].col());
                coeff_iter++;
            }
        }

        coeff_iter = 0;
        for (int k=0; k<linearMatrix_.outerSize(); ++k){
            for (SparseMatrix<double>::InnerIterator it(linearMatrix_,k); it; ++it){
            if(linearm_coeff_[coeff_iter].row() < Aeq.size1())
                it.valueRef() = (double)Aeq(linearm_coeff_[coeff_iter].row(), linearm_coeff_[coeff_iter].col());
            else
                it.valueRef() = (double)Aiq(linearm_coeff_[coeff_iter].row() - Aeq.size1(),linearm_coeff_[coeff_iter].col()) ;
            coeff_iter++;
            }
        }
        solver_.updateGradient(gradient_);
        solver_.updateHessianMatrix(hessian_);
        solver_.updateLinearConstraintsMatrix(linearMatrix_);
        solver_.updateBounds(lowerBound_,upperBound_);
    }
    solver_.solveProblem();
    sol_= solver_.getSolution();
    return sol_;
}