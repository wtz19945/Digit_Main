#include "mpc_solver.hpp"
using namespace Eigen;

MPC_Solver::MPC_Solver(int Cons_Num, int Vars_Num, int Pred_Num){
    Cons_Num_ = Cons_Num;
    Vars_Num_ = Vars_Num;
    Pred_Num_ = Pred_Num;
    QP_initialized_ = 0;

    hessian_.resize(Vars_Num_,Vars_Num_);
    linearMatrix_.resize(Cons_Num_,Vars_Num_);
    gradient_ = VectorXd::Zero(Vars_Num_,1);
    lowerBound_ = VectorXd::Zero(Cons_Num_,1);
    upperBound_ = VectorXd::Zero(Cons_Num_,1);
    sol_ = VectorXd::Zero(Vars_Num_,1);

    // Create gurobi solver
    env_ = std::make_unique<GRBEnv>();
    env_ -> start();
    model_ = std::make_unique<GRBModel>(*env_);
    
    // Add Gurobi variable
    for (int i = 0; i < Vars_Num; ++i) {
        if(i < Vars_Num_ - Pred_Num_ * 4) 
            vars_.push_back(model_->addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "cx" + std::to_string(i)));
        else
            vars_.push_back(model_->addVar(0.0, 1.0, 0.0, GRB_BINARY, "bx" + std::to_string(i)));
    }
    model_->update();
}

VectorXd MPC_Solver::Update_Solver(const casadi::DM& Aeq, const casadi::DM& beq,const casadi::DM& Aiq, 
                                    const casadi::DM& biq,const casadi::DM& H, const casadi::DM& f){



    if(QP_initialized_ == 0){
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
        QP_initialized_ = 1;

        // Add Gurobi Costs
        GRBQuadExpr qexpr;
        for(int i=0;i<Vars_Num_;i++){
            for(int j=0;j<Vars_Num_;j++){
                if((double)H(i,j) != 0){
                    qexpr.addTerm(0.5*(double)H(i,j), vars_[i], vars_[j]);
                }
            }
            qexpr.addTerm(gradient_(i), vars_[i]);
        }
        model_->setObjective(qexpr);

        // Add gurobi constraints
        for(int i=0;i<Aeq.size1() + Aiq.size1();i++){
            GRBLinExpr lhs = 0.0;
            for(int j=0; j<Vars_Num_; j++){
                if(i<Aeq.size1()){
                    if((double)Aeq(i,j) != 0)
                        lhs += (double)Aeq(i,j) * vars_[j];
                }
                else
                {
                    if((double)Aiq(i-Aeq.size1(),j) != 0)
                        lhs += (double)Aiq(i-Aeq.size1(),j) * vars_[j];
                }
            }
            if(i<Aeq.size1()){
                constraints_.push_back(model_->addConstr(lhs == upperBound_(i)));
            }
            else{
                constraints_.push_back(model_->addConstr(lhs <= upperBound_(i)));
            }
        }

        // Set model parameters
        model_->set(GRB_IntParam_OutputFlag, false);
        model_->set(GRB_IntParam_MIPFocus, 1); 
        model_->set(GRB_IntParam_DualReductions, 1); 
        
        //model_->set(GRB_DoubleParam_TimeLimit, 0.03); 
        //model_->set(GRB_DoubleParam_Heuristics, 0.05);
        //model_->set(GRB_IntParam_SubMIPNodes, 100);
        //model_->set(GRB_IntParam_PoolSolutions, 10);
        //model_->set(GRB_IntParam_PoolSearchMode, 1);
        //model_->set(GRB_DoubleParam_FeasibilityTol, 1e-4);
        //model_->set(GRB_DoubleParam_MIPGap, 0.001);
        //model_->set(GRB_IntParam_TuneResults, 15);
    }
    else{
        // update Gurobi 
        GRBQuadExpr qexpr;
        for(int i=0;i<Vars_Num_;i++){
            // warm start solver
            if(i < Vars_Num_ - Pred_Num_ * 4)
                vars_[i].set(GRB_DoubleAttr_Start, sol_[i]);
            // Add linear cost
            qexpr.addTerm((double)f(i), vars_[i]);
            //qexpr.addTerm(1e-4, vars_[i], vars_[i]);
        }
        // Add quadratic cost
        int coeff_iter = 0;
        for (int k=0; k<hessian_.outerSize(); ++k){
            for (SparseMatrix<double>::InnerIterator it(hessian_,k); it; ++it){
                qexpr.addTerm(0.5*(double)H(hessian_coeff_[coeff_iter].row(),hessian_coeff_[coeff_iter].col()), vars_[hessian_coeff_[coeff_iter].row()], vars_[hessian_coeff_[coeff_iter].col()]);
                coeff_iter++;
            }
        }
        model_->setObjective(qexpr);

        // update constraints
        for(int i=0;i<Cons_Num_;i++){
            if(i < Aeq.size1())
                constraints_[i].set(GRB_DoubleAttr_RHS, (double)beq(i));
            else
                constraints_[i].set(GRB_DoubleAttr_RHS, (double)biq(i - Aeq.size1()));
        }
        coeff_iter = 0;
        for (int k=0; k<linearMatrix_.outerSize(); ++k){
            for (SparseMatrix<double>::InnerIterator it(linearMatrix_,k); it; ++it){
                int row = linearm_coeff_[coeff_iter].row();
                int col = linearm_coeff_[coeff_iter].col();
                if(linearm_coeff_[coeff_iter].row() < Aeq.size1())
                    model_->chgCoeff(constraints_[row], vars_[col], (double)Aeq(row, col));
                else
                    model_->chgCoeff(constraints_[row], vars_[col], (double)Aiq(row - Aeq.size1(), col));
                coeff_iter++;
            }
        }
    }

    model_->optimize();
    if (model_->get(GRB_IntAttr_Status) == GRB_OPTIMAL || model_->get(GRB_IntAttr_Status) == GRB_TIME_LIMIT) {
        if(model_->get(GRB_DoubleAttr_Runtime)  >= 0.02){
            //std::cout << "need more solve time: " << model_->get(GRB_DoubleAttr_Runtime) << std::endl;
        }
        for(int i = 0; i< Vars_Num_; i++)
            sol_(i) = vars_[i].get(GRB_DoubleAttr_X);
    }
    else{
        std::cout << "error status: " << model_->get(GRB_IntAttr_Status) << std::endl;
    }

    return sol_;
}