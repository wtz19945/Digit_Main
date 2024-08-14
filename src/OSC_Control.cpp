#include "OSC_Control.hpp"

OSC_Control::OSC_Control(std::shared_ptr<cpptoml::table> config){
    Wcom = config->get_qualified_as<double>("QP-Params.com_W").value_or(0);
    Wcomx = config->get_qualified_as<double>("QP-Params.com_Wx").value_or(0);
    Wcomy = config->get_qualified_as<double>("QP-Params.com_Wy").value_or(0);
    Wcomz = config->get_qualified_as<double>("QP-Params.com_Wz").value_or(0);
    Wcomrz = config->get_qualified_as<double>("QP-Params.com_Wrz").value_or(0);
    Wcomry = config->get_qualified_as<double>("QP-Params.com_Wry").value_or(0);
    Wcomrx = config->get_qualified_as<double>("QP-Params.com_Wrx").value_or(0);
    
    double Wcomxst = config->get_qualified_as<double>("QP-Params.com_Wxst").value_or(0);
    double Wcomyst = config->get_qualified_as<double>("QP-Params.com_Wyst").value_or(0);
    double Wcomzst = config->get_qualified_as<double>("QP-Params.com_Wzst").value_or(0);
    double Wcomrzst = config->get_qualified_as<double>("QP-Params.com_Wrzst").value_or(0);
    double Wcomryst = config->get_qualified_as<double>("QP-Params.com_Wryst").value_or(0);
    double Wcomrxst = config->get_qualified_as<double>("QP-Params.com_Wrxst").value_or(0);

    Wff  = config->get_qualified_as<double>("QP-Params.st_foot_W").value_or(0);
    Wfb  = config->get_qualified_as<double>("QP-Params.st_foot_W").value_or(0);
    
    Wffsw  = config->get_qualified_as<double>("QP-Params.sw_foot_W").value_or(0);
    Wfbsw  = config->get_qualified_as<double>("QP-Params.sw_foot_W").value_or(0);
    double Wswx  = config->get_qualified_as<double>("QP-Params.sw_foot_Wx").value_or(0);
    double Wswy  = config->get_qualified_as<double>("QP-Params.sw_foot_Wy").value_or(0);

    force_max = config->get_qualified_as<double>("QP-Params.force_max").value_or(0);
    mu = config->get_qualified_as<double>("QP-Params.mu").value_or(0);
    
    Weight_ToeF = Wff*MatrixXd::Identity(6,6);
    Weight_ToeB = Wfb*MatrixXd::Identity(8,8);

    Weight_ToeFsw = Wffsw*MatrixXd::Identity(6,6);
    Weight_ToeBsw = Wfbsw*MatrixXd::Identity(8,8);
    Weight_ToeFsw(0,0) = Wswx;
    Weight_ToeFsw(3,3) = Wswx;
    Weight_ToeFsw(1,1) = Wswy;
    Weight_ToeFsw(4,4) = Wswy;

    Weight_ToeBsw(0,0) = Wswx;
    Weight_ToeBsw(4,4) = Wswx;
    Weight_ToeBsw(1,1) = Wswy;
    Weight_ToeBsw(5,5) = Wswy;
/*  Weight_ToeFsw(1,1) *= 0.1;
    Weight_ToeFsw(4,4) *= 0.1;

    Weight_ToeBsw(1,1) *= 0.1;
    Weight_ToeBsw(5,5) *= 0.1;

    Weight_ToeFsw(2,2) *= 2.5;
    Weight_ToeFsw(5,5) *= 2.5;

    Weight_ToeBsw(2,2) *= 2.5;
    Weight_ToeBsw(6,6) *= 2.5; */
/* 
    Weight_ToeFsw(2,2) *= 5;
    Weight_ToeFsw(5,5) *= 5; 
    Weight_ToeBsw(2,2) *= 5;
    Weight_ToeBsw(6,6) *= 5; */
    Weight_pel_ = Wcomx*MatrixXd::Identity(6,6);
    Weight_pel_(0,0) = Wcomx;
    Weight_pel_(1,1) = Wcomy;
    Weight_pel_(2,2) = Wcomz;
    Weight_pel_(3,3) = Wcomrz;
    Weight_pel_(4,4) = Wcomry;
    Weight_pel_(5,5) = Wcomrx;

    Weight_pelst = Wcomx*MatrixXd::Identity(6,6);
    Weight_pelst(0,0) = Wcomxst;
    Weight_pelst(1,1) = Wcomyst;
    Weight_pelst(2,2) = Wcomzst;
    Weight_pelst(3,3) = Wcomrzst;
    Weight_pelst(4,4) = Wcomryst;
    Weight_pelst(5,5) = Wcomrxst;

    // initialize limit vector
    ddq_limit = VectorXd::Zero(20,1);     // state acceleration 
    u_limit = VectorXd::Zero(12,1);       // torque limits
    tor_limit = VectorXd::Zero(4,1);      // generalized force
    f_limit_max = VectorXd::Zero(12,1);   // contact force
    f_limit_min = VectorXd::Zero(12,1);   // contact force
    f_cons_min = VectorXd::Zero(16,1);    // friction constraint limit
    qt = VectorXd::Zero(17,1);            // task space acc limit
    
    for(int i=0;i<20;i++){
        ddq_limit(i) = OsqpEigen::INFTY;
    }
    for(int i=0;i<16;i++){
        f_cons_min(i) = -OsqpEigen::INFTY;
    }
    for(int i=0;i<17;i++){
        qt(i) = OsqpEigen::INFTY;
    }

    u_limit  << 116.682, 70.1765, 206.928,220.928,35.9759,35.9759, 116.682, 70.1765, 206.928,220.928,35.9759,35.9759;
    tor_limit<< OsqpEigen::INFTY,  OsqpEigen::INFTY,  OsqpEigen::INFTY,  OsqpEigen::INFTY;

    for(int i=0;i<12;i++){
      f_limit_max(i) = force_max * mu;
      f_limit_min(i) = -force_max * mu;
    }
    f_limit_max(2) = force_max;
    f_limit_max(5) = force_max;
    f_limit_max(8) = force_max;
    f_limit_max(11) = force_max;
    f_limit_min(2) = 0;
    f_limit_min(5) = 0;
    f_limit_min(8) = 0;
    f_limit_min(11) = 0;

    // QP Size
    Vars_Num = 20 + 12 + 4 + 12 + 17;
    Cons_Num = 20 + 4 + Vars_Num + 16 + 17;

    // QP Matrix
    constraint_full = MatrixXd::Zero(Cons_Num,Vars_Num);
    hessian_full = MatrixXd::Zero(Vars_Num,Vars_Num);
    linearMatrix.resize(Cons_Num,Vars_Num);
    hessian.resize(Vars_Num,Vars_Num);

    // QP Vector
    gradient = VectorXd::Zero(Vars_Num,1);
    lowerBound = VectorXd::Zero(Cons_Num,1);
    upperBound = VectorXd::Zero(Cons_Num,1);
    QPSolution_ = VectorXd::Zero(Vars_Num,1);

    // Set Up Solver
    solver.data()->setNumberOfVariables(Vars_Num);
    solver.data()->setNumberOfConstraints(Cons_Num);
}

void OSC_Control::setupQPVector(VectorXd des_acc_pel, VectorXd des_acc, VectorXd des_acc_toe, VectorXd G, VectorXd contact){
    double contact1 = contact(0);
    double contact2 = contact(1);

    if(contact1 == 1 && contact2 == 1){
        gradient << VectorXd::Zero(3,1), -Weight_pel_.block(3,3,3,3) * des_acc_pel.block(3,0,3,1), VectorXd::Zero(42,1),-Weight_ToeF.block(0,0,3,3) * des_acc.block(0,0,3,1),
                    -Weight_ToeB.block(0,0,4,4) * des_acc_toe.block(0,0,4,1),
                    -Weight_ToeF.block(3,3,3,3) * des_acc.block(3,0,3,1),
                    -Weight_ToeB.block(4,4,4,4) * des_acc_toe.block(4,0,4,1),
                    -Weight_pel_.block(0,0,3,3) * des_acc_pel.block(0,0,3,1);
    }
    else if(contact1 == 0 && contact2 == 1){
        gradient << VectorXd::Zero(3,1),-Weight_pelst.block(3,3,3,3) * des_acc_pel.block(3,0,3,1), VectorXd::Zero(42,1),-Weight_ToeFsw.block(0,0,3,3) * des_acc.block(0,0,3,1),
                    -Weight_ToeBsw.block(0,0,4,4) * des_acc_toe.block(0,0,4,1),
                    -Weight_ToeF.block(3,3,3,3) * des_acc.block(3,0,3,1),
                    -Weight_ToeB.block(4,4,4,4) * des_acc_toe.block(4,0,4,1),
                    -Weight_pelst.block(0,0,3,3) * des_acc_pel.block(0,0,3,1);
    }
    else if(contact1 == 1 && contact2 == 0){
        gradient << VectorXd::Zero(3,1), -Weight_pelst.block(3,3,3,3) * des_acc_pel.block(3,0,3,1), VectorXd::Zero(42,1),-Weight_ToeF.block(0,0,3,3) * des_acc.block(0,0,3,1),
                    -Weight_ToeB.block(0,0,4,4) * des_acc_toe.block(0,0,4,1),
                    -Weight_ToeFsw.block(3,3,3,3) * des_acc.block(3,0,3,1),
                    -Weight_ToeBsw.block(4,4,4,4) * des_acc_toe.block(4,0,4,1),
                    -Weight_pelst.block(0,0,3,3) * des_acc_pel.block(0,0,3,1);
    }
    else{
        gradient << VectorXd::Zero(3,1), -Weight_pel_.block(3,3,3,3) * des_acc_pel.block(3,0,3,1), VectorXd::Zero(42,1),-Weight_ToeF.block(0,0,3,3) * des_acc.block(0,0,3,1),
                    -Weight_ToeB.block(0,0,4,4) * des_acc_toe.block(0,0,4,1),
                    -Weight_ToeF.block(3,3,3,3) * des_acc.block(3,0,3,1),
                    -Weight_ToeB.block(4,4,4,4) * des_acc_toe.block(4,0,4,1),
                    -Weight_pel_.block(0,0,3,3) * des_acc_pel.block(0,0,3,1);
    }
    
    f_limit_max(2) = force_max * contact1;
    f_limit_max(5) = force_max * contact1;
    f_limit_max(8) = force_max * contact2;
    f_limit_max(11) = force_max * contact2;
    lowerBound << -G, VectorXd::Zero(4,1) , -ddq_limit, -u_limit, -tor_limit, f_limit_min, -qt,
                f_cons_min,VectorXd::Zero(17,1);

    upperBound << -G, VectorXd::Zero(4,1) ,  ddq_limit,  u_limit, tor_limit , f_limit_max, qt, 
                VectorXd::Zero(16,1),VectorXd::Zero(17,1);
}

void OSC_Control::updateQPVector(const VectorXd& des_acc_pel, const VectorXd& des_acc, const VectorXd& des_acc_toe, const VectorXd& G, const VectorXd& contact){
    // currently, QP vector is basically changing with every iteration. no difference between setup and update
    setupQPVector(des_acc_pel, des_acc, des_acc_toe, G,contact);
}

void OSC_Control::setupQPMatrix(MatrixXd Weight_pel, MatrixXd M, MatrixXd B, MatrixXd Spring_Jaco,
        MatrixXd left_toe_jaco_fa, MatrixXd left_toe_back_jaco_fa, MatrixXd right_toe_jaco_fa, MatrixXd right_toe_back_jaco_fa,
        MatrixXd left_toe_rot_jaco_fa, MatrixXd right_toe_rot_jaco_fa){
    // This function should be only called at initialization stage as it iterates through the entire matrix to record sparsity of the matrix. 
    // Hessian matrix
    hessian_full.block(3,3,3,3) = Weight_pel_.block(3,3,3,3);
    hessian_full.block(48,48,3,3) = Weight_ToeF.block(0,0,3,3);
    hessian_full.block(51,51,4,4) = Weight_ToeB.block(0,0,4,4);
    hessian_full.block(55,55,3,3) = Weight_ToeF.block(3,3,3,3);
    hessian_full.block(58,58,4,4) = Weight_ToeB.block(4,4,4,4);
    hessian_full.block(62,62,3,3) = Weight_pel_.block(0,0,3,3);

    for(int i = 0;i<hessian_full.rows();i++){
        hessian.insert(i,i) = hessian_full(i,i) + 1e-3;
    }

    MatrixXd Jcom = MatrixXd::Zero(3,20);
    Jcom.block(0,0,3,3) = MatrixXd::Identity(3,3);

    // Constraint matrix
    constraint_full.block(0,0,20,20) = M;
    constraint_full.block(0,20,20,12) = -B;
    constraint_full.block(0,32,20,4) = -(Spring_Jaco).transpose();
    constraint_full.block(0,36,20,3) = -left_toe_jaco_fa.transpose();
    constraint_full.block(0,39,20,3) = -left_toe_back_jaco_fa.transpose();
    constraint_full.block(0,42,20,3) = -right_toe_jaco_fa.transpose();
    constraint_full.block(0,45,20,3) = -right_toe_back_jaco_fa.transpose();
    constraint_full.block(20,0,4,20) = Spring_Jaco;
    constraint_full.block(24,0,Vars_Num,Vars_Num) = MatrixXd::Identity(Vars_Num,Vars_Num);
    constraint_full.block(24+Vars_Num,Vars_Num-29,16,12) = get_fric_cons(mu);
    constraint_full.block(40+Vars_Num,0,3,20) = left_toe_jaco_fa;
    constraint_full.block(43+Vars_Num,0,4,20) = left_toe_rot_jaco_fa;
    constraint_full.block(47+Vars_Num,0,3,20) = right_toe_jaco_fa;
    constraint_full.block(50+Vars_Num,0,4,20) = right_toe_rot_jaco_fa;
    constraint_full.block(54+Vars_Num,0,3,20) = Jcom;
    constraint_full.block(40+Vars_Num,Vars_Num-17,17,17) = -MatrixXd::Identity(17,17);

    for(int i = 0;i<constraint_full.cols();i++){
        for(int j = 0;j<constraint_full.rows();j++){
            if(abs(constraint_full(j,i)) > 1e-4){
                coefficients.push_back(Eigen::Triplet<double>(j,i,constraint_full(j,i)));
                linearMatrix.insert(j,i) = constraint_full(j,i);
            }
        }
    }
}

void OSC_Control::updateQPMatrix(const MatrixXd& Weight_pel, const MatrixXd& M, const MatrixXd& B, const MatrixXd& Spring_Jaco,
        const MatrixXd& left_toe_jaco_fa, const MatrixXd& left_toe_back_jaco_fa, const MatrixXd& right_toe_jaco_fa, const MatrixXd& right_toe_back_jaco_fa,
        const MatrixXd& left_toe_rot_jaco_fa, const MatrixXd& right_toe_rot_jaco_fa, const VectorXd& contact){
    
    double contact1 = contact(0);
    double contact2 = contact(1);
    // Hessian matrix
    if(contact1 == 1 && contact2 == 1){
        hessian_full.block(3,3,3,3) = Weight_pel_.block(3,3,3,3);
        hessian_full.block(48,48,3,3) = Weight_ToeF.block(0,0,3,3);
        hessian_full.block(51,51,4,4) = Weight_ToeB.block(0,0,4,4);
        hessian_full.block(55,55,3,3) = Weight_ToeF.block(3,3,3,3);
        hessian_full.block(58,58,4,4) = Weight_ToeB.block(4,4,4,4);
        hessian_full.block(62,62,3,3) = Weight_pel_.block(0,0,3,3);
    }
    else if(contact1 == 0 && contact2 == 1){
        hessian_full.block(3,3,3,3) = Weight_pelst.block(3,3,3,3);
        hessian_full.block(48,48,3,3) = Weight_ToeFsw.block(0,0,3,3);
        hessian_full.block(51,51,4,4) = Weight_ToeBsw.block(0,0,4,4);
        hessian_full.block(55,55,3,3) = Weight_ToeF.block(3,3,3,3);
        hessian_full.block(58,58,4,4) = Weight_ToeB.block(4,4,4,4);
        hessian_full.block(62,62,3,3) = Weight_pelst.block(0,0,3,3);
    }
    else if(contact1 == 1 && contact2 == 0){
        hessian_full.block(3,3,3,3) = Weight_pelst.block(3,3,3,3);
        hessian_full.block(48,48,3,3) = Weight_ToeF.block(0,0,3,3);
        hessian_full.block(51,51,4,4) = Weight_ToeB.block(0,0,4,4);
        hessian_full.block(55,55,3,3) = Weight_ToeFsw.block(3,3,3,3);
        hessian_full.block(58,58,4,4) = Weight_ToeBsw.block(4,4,4,4);
        hessian_full.block(62,62,3,3) = Weight_pelst.block(0,0,3,3);
    }
    else{
        hessian_full.block(3,3,3,3) = Weight_pel_.block(3,3,3,3);
        hessian_full.block(48,48,3,3) = Weight_ToeF.block(0,0,3,3);
        hessian_full.block(51,51,4,4) = Weight_ToeB.block(0,0,4,4);
        hessian_full.block(55,55,3,3) = Weight_ToeF.block(3,3,3,3);
        hessian_full.block(58,58,4,4) = Weight_ToeB.block(4,4,4,4);
        hessian_full.block(62,62,3,3) = Weight_pel_.block(0,0,3,3);
    }

    for(int i = 0;i < hessian_full.rows();i++){
      if((i>=3&&i<6)||i>=48){
        hessian.coeffRef(i,i) = hessian_full(i,i);
      }
      else{
        hessian.coeffRef(i,i) = 1e-2;
      }
    }

    // Constraint matrix. Only update changing parts
    constraint_full.block(0,0,20,20) = M;
    constraint_full.block(0,20,20,12) = -B;
    constraint_full.block(0,36,20,3) = -left_toe_jaco_fa.transpose();
    constraint_full.block(0,39,20,3) = -left_toe_back_jaco_fa.transpose();
    constraint_full.block(0,42,20,3) = -right_toe_jaco_fa.transpose();
    constraint_full.block(0,45,20,3) = -right_toe_back_jaco_fa.transpose();
    constraint_full.block(40+Vars_Num,0,3,20) = left_toe_jaco_fa;
    constraint_full.block(43+Vars_Num,0,4,20) = left_toe_rot_jaco_fa;
    constraint_full.block(47+Vars_Num,0,3,20) = right_toe_jaco_fa;
    constraint_full.block(50+Vars_Num,0,4,20) = right_toe_rot_jaco_fa;

    // Speed up setting constraint matrix by assuming all non-zeros' indexes are fixed
/*     int coeff_iter = 0;
    for (int k=0; k<linearMatrix.outerSize(); ++k){
        for (SparseMatrix<double>::InnerIterator it(linearMatrix,k); it; ++it){
            it.valueRef() = constraint_full(coefficients[coeff_iter].row(),coefficients[coeff_iter].col());
            coeff_iter++;
        }
    }
     */
    int coeff_iter = 0;
    for (int k = 0; k < linearMatrix.outerSize(); ++k) {
        for (SparseMatrix<double>::InnerIterator it(linearMatrix, k); it; ++it) {
            linearMatrix.coeffRef(it.row(), it.col()) = constraint_full(coefficients[coeff_iter].row(), coefficients[coeff_iter].col());
            coeff_iter++;
        }
    }

}

// Track CoM instead of base origin
void OSC_Control::setupQPMatrixCoM(MatrixXd Weight_pel, MatrixXd M, MatrixXd B, MatrixXd Spring_Jaco,
        MatrixXd left_toe_jaco_fa, MatrixXd left_toe_back_jaco_fa, MatrixXd right_toe_jaco_fa, MatrixXd right_toe_back_jaco_fa,
        MatrixXd left_toe_rot_jaco_fa, MatrixXd right_toe_rot_jaco_fa, MatrixXd Jcom){
    // This function should be only called at initialization stage as it iterates through the entire matrix to record sparsity of the matrix. 
    // Hessian matrix
    hessian_full.block(3,3,3,3) = Weight_pel_.block(3,3,3,3);
    hessian_full.block(48,48,3,3) = Weight_ToeF.block(0,0,3,3);
    hessian_full.block(51,51,4,4) = Weight_ToeB.block(0,0,4,4);
    hessian_full.block(55,55,3,3) = Weight_ToeF.block(3,3,3,3);
    hessian_full.block(58,58,4,4) = Weight_ToeB.block(4,4,4,4);
    hessian_full.block(62,62,3,3) = Weight_pel_.block(0,0,3,3);

    for(int i = 0;i<hessian_full.rows();i++){
        hessian.insert(i,i) = hessian_full(i,i) + 1e-3;
    }
    // Constraint matrix
    constraint_full.block(0,0,20,20) = M;
    constraint_full.block(0,20,20,12) = -B;
    constraint_full.block(0,32,20,4) = -(Spring_Jaco).transpose();
    constraint_full.block(0,36,20,3) = -left_toe_jaco_fa.transpose();
    constraint_full.block(0,39,20,3) = -left_toe_back_jaco_fa.transpose();
    constraint_full.block(0,42,20,3) = -right_toe_jaco_fa.transpose();
    constraint_full.block(0,45,20,3) = -right_toe_back_jaco_fa.transpose();
    constraint_full.block(20,0,4,20) = Spring_Jaco;
    constraint_full.block(24,0,Vars_Num,Vars_Num) = MatrixXd::Identity(Vars_Num,Vars_Num);
    constraint_full.block(24+Vars_Num,Vars_Num-29,16,12) = get_fric_cons(mu);
    constraint_full.block(40+Vars_Num,0,3,20) = left_toe_jaco_fa;
    constraint_full.block(43+Vars_Num,0,4,20) = left_toe_rot_jaco_fa;
    constraint_full.block(47+Vars_Num,0,3,20) = right_toe_jaco_fa;
    constraint_full.block(50+Vars_Num,0,4,20) = right_toe_rot_jaco_fa;
    constraint_full.block(54+Vars_Num,0,3,20) = Jcom;
    constraint_full.block(40+Vars_Num,Vars_Num-17,17,17) = -MatrixXd::Identity(17,17);

    for(int i = 0;i<constraint_full.cols();i++){
        for(int j = 0;j<constraint_full.rows();j++){
            if(abs(constraint_full(j,i)) > 1e-4){
                coefficients.push_back(Eigen::Triplet<double>(j,i,constraint_full(j,i)));
                linearMatrix.insert(j,i) = constraint_full(j,i);
            }
        }
    }
}

void OSC_Control::updateQPMatrixCoM(MatrixXd Weight_pel, MatrixXd M, MatrixXd B, MatrixXd Spring_Jaco,
        MatrixXd left_toe_jaco_fa, MatrixXd left_toe_back_jaco_fa, MatrixXd right_toe_jaco_fa, MatrixXd right_toe_back_jaco_fa,
        MatrixXd left_toe_rot_jaco_fa, MatrixXd right_toe_rot_jaco_fa, VectorXd contact, MatrixXd Jcom){
    
    double contact1 = contact(0);
    double contact2 = contact(1);
    // Hessian matrix
    if(contact1 == 1 && contact2 == 1){
        hessian_full.block(3,3,3,3) = Weight_pel_.block(3,3,3,3);
        hessian_full.block(48,48,3,3) = Weight_ToeF.block(0,0,3,3);
        hessian_full.block(51,51,4,4) = Weight_ToeB.block(0,0,4,4);
        hessian_full.block(55,55,3,3) = Weight_ToeF.block(3,3,3,3);
        hessian_full.block(58,58,4,4) = Weight_ToeB.block(4,4,4,4);
        hessian_full.block(62,62,3,3) = Weight_pel_.block(0,0,3,3);
    }
    else if(contact1 == 0 && contact2 == 1){
        hessian_full.block(3,3,3,3) = Weight_pelst.block(3,3,3,3);
        hessian_full.block(48,48,3,3) = Weight_ToeFsw.block(0,0,3,3);
        hessian_full.block(51,51,4,4) = Weight_ToeBsw.block(0,0,4,4);
        hessian_full.block(55,55,3,3) = Weight_ToeF.block(3,3,3,3);
        hessian_full.block(58,58,4,4) = Weight_ToeB.block(4,4,4,4);
        hessian_full.block(62,62,3,3) = Weight_pelst.block(0,0,3,3);
    }
    else if(contact1 == 1 && contact2 == 0){
        hessian_full.block(3,3,3,3)= Weight_pelst.block(3,3,3,3);
        hessian_full.block(48,48,3,3) = Weight_ToeF.block(0,0,3,3);
        hessian_full.block(51,51,4,4) = Weight_ToeB.block(0,0,4,4);
        hessian_full.block(55,55,3,3) = Weight_ToeFsw.block(3,3,3,3);
        hessian_full.block(58,58,4,4) = Weight_ToeBsw.block(4,4,4,4);
        hessian_full.block(62,62,3,3) = Weight_pelst.block(0,0,3,3);
    }
    else{
        hessian_full.block(3,3,3,3) = Weight_pel_.block(3,3,3,3);
        hessian_full.block(48,48,3,3) = Weight_ToeF.block(0,0,3,3);
        hessian_full.block(51,51,4,4) = Weight_ToeB.block(0,0,4,4);
        hessian_full.block(55,55,3,3) = Weight_ToeF.block(3,3,3,3);
        hessian_full.block(58,58,4,4) = Weight_ToeB.block(4,4,4,4);
        hessian_full.block(62,62,3,3) = Weight_pel_.block(0,0,3,3);
    }

    for(int i = 0;i<hessian_full.rows();i++){
      if((i>=3&&i<6)||i>=48){
        hessian.coeffRef(i,i) = hessian_full(i,i);
      }
      else{
        hessian.coeffRef(i,i) = 1e-3;
      }
    }

    // Constraint matrix. Only update changing parts
    constraint_full.block(0,0,20,20) = M;
    constraint_full.block(0,20,20,12) = -B;
    constraint_full.block(0,36,20,3) = -left_toe_jaco_fa.transpose();
    constraint_full.block(0,39,20,3) = -left_toe_back_jaco_fa.transpose();
    constraint_full.block(0,42,20,3) = -right_toe_jaco_fa.transpose();
    constraint_full.block(0,45,20,3) = -right_toe_back_jaco_fa.transpose();
    constraint_full.block(40+Vars_Num,0,3,20) = left_toe_jaco_fa;
    constraint_full.block(43+Vars_Num,0,4,20) = left_toe_rot_jaco_fa;
    constraint_full.block(47+Vars_Num,0,3,20) = right_toe_jaco_fa;
    constraint_full.block(50+Vars_Num,0,4,20) = right_toe_rot_jaco_fa;
    constraint_full.block(54+Vars_Num,0,3,20) = Jcom;
    // Speed up setting constraint matrix by assuming all non-zeros' indexes are fixed
/*     int coeff_iter = 0;
    for (int k=0; k<linearMatrix.outerSize(); ++k){
        for (SparseMatrix<double>::InnerIterator it(linearMatrix,k); it; ++it){
            it.valueRef() = constraint_full(coefficients[coeff_iter].row(),coefficients[coeff_iter].col());
            coeff_iter++;
        }
    }
     */
    int coeff_iter = 0;
    for (int k = 0; k < linearMatrix.outerSize(); ++k) {
        for (SparseMatrix<double>::InnerIterator it(linearMatrix, k); it; ++it) {
            linearMatrix.coeffRef(it.row(), it.col()) = constraint_full(coefficients[coeff_iter].row(), coefficients[coeff_iter].col());
            coeff_iter++;
        }
    }

}

void OSC_Control::setUpQP(bool mute_solver){
    solver.settings()->setVerbosity(mute_solver);
    solver.settings()->setWarmStart(true);
    solver.settings()->setCheckTermination(10);
    solver.settings()->setRho(1e-3);
    solver.settings()->setAlpha(0.6);
    solver.settings()->setAdaptiveRho(true);
    solver.settings()->setAbsoluteTolerance(1e-4);
    solver.settings()->setRelativeTolerance(1e-4);
    solver.settings()->setPrimalInfeasibilityTolerance(1e-5);
    solver.settings()->setDualInfeasibilityTolerance(1e-5);
    //solver.settings()->setMaxIteration(20);
    solver.data()->setHessianMatrix(hessian);
    solver.data()->setGradient(gradient);
    solver.data()->setLinearConstraintsMatrix(linearMatrix);
    solver.data()->setLowerBound(lowerBound);
    solver.data()->setUpperBound(upperBound);
    solver.initSolver();
}

void OSC_Control::muteSolver(bool mute_solver){
    solver.settings()->setVerbosity(true);
}

void OSC_Control::updateQP(){
    solver.updateGradient(gradient);
    solver.updateHessianMatrix(hessian);
    solver.updateLinearConstraintsMatrix(linearMatrix);
    solver.updateBounds(lowerBound,upperBound);
}

VectorXd OSC_Control::solveQP(){
    solver.solveProblem();
    if((solver.getStatus() == OsqpEigen::Status::Solved)){
        QPSolution_ = solver.getSolution();
    }
    return QPSolution_;
}

MatrixXd OSC_Control::get_fric_cons(double mu){
  MatrixXd fric_cons = MatrixXd::Zero(16,12);
  MatrixXd fric_sub  = MatrixXd::Zero(4,3);
  fric_sub << 1,1,-mu,1,-1,-mu,-1,1,-mu,-1,-1,-mu;
  for(int i=0;i<4;i++){
    fric_cons.block(4*i,3*i,4,3) = fric_sub;
  }
  return fric_cons;
}