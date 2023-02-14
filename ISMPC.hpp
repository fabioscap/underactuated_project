#ifndef JOINTISMPC_ISMPC_HPP
#define JOINTISMPC_ISMPC_HPP

#include <iostream>
#include <vector>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <labrob_qpsolvers/qpsolvers.hpp>

#include "parameters.cpp"
#include "types.hpp"
#include "utils.cpp"
#include "FootstepPlan.hpp"

#include "dart/dart.hpp"

class ISMPC {

public:
  ISMPC(FootstepPlan* _footstepPlanner, State initial_state, dart::dynamics::SkeletonPtr mRobot);
  State MPC(WalkState walkState, State Current);

private:



  int n_dof_;

  State state;
  Eigen::Vector4d footstepPredicted;

  dart::dynamics::SkeletonPtr robot;

  // Neutral configuration of the robot
  Eigen::VectorXd q_neutral_;

  Eigen::MatrixXd rk_;
  Eigen::MatrixXd rk_prev_; // used for the truncated tail
  Eigen::MatrixXd summation_;
  Eigen::MatrixXd summation_pred_;

  // Stability constraint
  Eigen::MatrixXd A_stab_;
  Eigen::VectorXd b_stab_;

  // ZMP constraint
  Eigen::MatrixXd A_zmp_;
  Eigen::VectorXd b_zmp_L_;
  Eigen::VectorXd b_zmp_R_;


  Eigen::VectorXd kD_lim_L_;
  Eigen::VectorXd kD_lim_R_;


  // From Old IS-MPC
  //***********************
    // Matrices for prediction
    Eigen::VectorXd p;
    Eigen::MatrixXd P;
    Eigen::MatrixXd Vu;
    Eigen::MatrixXd Vs;

    // Matrices for cost function
    Eigen::MatrixXd costFunctionH = Eigen::MatrixXd::Zero(2*(N+M),2*(N+M));
    Eigen::VectorXd costFunctionF = Eigen::VectorXd::Zero(2*(N+M));

    // Matrices for stability constraint
    Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(2,(N*2)+(M*2));
    Eigen::VectorXd beq = Eigen::VectorXd::Zero(2);

    //Matrices for balance constraint
    Eigen::MatrixXd AZmp = Eigen::MatrixXd::Zero(2*N,2*(N+M));
    Eigen::VectorXd bZmpMax = Eigen::VectorXd::Zero(2*N);
    Eigen::VectorXd bZmpMin = Eigen::VectorXd::Zero(2*N);

    // Matrices for feasibility constraints
    Eigen::MatrixXd AFootsteps = Eigen::MatrixXd::Zero(2*M,2*(M+N));
    Eigen::VectorXd bFootstepsMax = Eigen::VectorXd::Zero(2*M);
    Eigen::VectorXd bFootstepsMin = Eigen::VectorXd::Zero(2*M);

    // Matrices for the stacked constraints
    Eigen::MatrixXd AConstraint;
    Eigen::VectorXd bConstraintMax;
    Eigen::VectorXd bConstraintMin;

    // Footstep plan
    std::shared_ptr<FootstepPlan> plan;
  //***********************

  Eigen::VectorXd supportFootPose = Eigen::VectorXd::Zero(6);

  //Logging
  Eigen::VectorXd pred_zmp;

  std::shared_ptr<labrob::qpsolvers::QPSolverEigenWrapper<double>> ISMPC_qp_solver_ptr_;
  std::shared_ptr<labrob::qpsolvers::QPSolverEigenWrapper<double>> IK_qp_solver_ptr_;


};

#endif // JOINTISMPC_ISMPC_HPP
