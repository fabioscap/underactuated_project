#include "ISMPC.hpp"


ISMPC::ISMPC(FootstepPlan* _footstepPlanner, State initial_state, dart::dynamics::SkeletonPtr mRobot) :
  plan(_footstepPlanner),
  state(initial_state),
  robot(mRobot)
{
  ISMPC_qp_solver_ptr_ = std::make_shared<labrob::qpsolvers::QPSolverEigenWrapper<double>>(
      //std::make_shared<labrob::qpsolvers::HPIPMQPSolver>(2*N+2*M, 2, 2*N+2*M));
      std::make_shared<labrob::qpsolvers::HPIPMQPSolver>(2*N+2*M, 2, 2*N));
}

State ISMPC::MPC(WalkState walkState, State current) {

    // Loop closure
    state.com.pos = current.com.pos;
    //state.com.vel = current.com.vel;

    // Matrices for ZMP prediction
    p = Eigen::VectorXd::Ones(N);
    P = Eigen::MatrixXd::Zero(N,N);

    for(int i = 0; i < N; ++i)
    	for(int j = 0; j <= i; ++j)
            P(i, j) = timeStep;

    // Reset constraint matrices
    AZmp.setZero();
    AFootsteps.setZero();

    // Get the pose of the support foot in the world frame
    Eigen::VectorXd supportFootPose(6); //FIXME this is the openloop pose of the support foot
    supportFootPose << plan->getFootstepPosition(walkState.footstepCounter), 0.0, 0.0, plan->getFootstepOrientation(walkState.footstepCounter);

    // Construct some matrices that will be used later in the cost function and constraints
    // ************************************************************************************

    // Construct the Cc matrix, which maps iterations to predicted/previewed footsteps
    Eigen::MatrixXd CcFull = Eigen::MatrixXd::Zero(N,M+1);

    int fsAhead = 0;
    int innerIter = walkState.iter % (S + D);
    for (int i = 0; i < N; ++i) { // we iterate over control + preview
	fsAhead = (innerIter + i) / (S + D); // integer division !!

        if ((walkState.iter + i) % (S + D) < S) { // we are in single support
            CcFull(i, fsAhead) = 1;
        } else {
            CcFull(i, fsAhead) = 1.0 - (double)((walkState.iter + i) % (S + D) - S) / (double)D;
            CcFull(i, fsAhead + 1) = (double)((walkState.iter + i) % (S + D) - S) / (double)D;
        }
    }

    // Split the matrix: currentFootstepZmp is for the current, Cc for all predicted footsteps
    Eigen::VectorXd currentFootstepZmp = CcFull.block(0,0,N,1);
    Eigen::MatrixXd Cc = CcFull.block(0,1,N,M);

    // Construct the Ic matrix, which removes constraints from double support phases
    Eigen::MatrixXd Ic = Eigen::MatrixXd::Identity(N,N);
    for (int i = 0; i < N; ++i) {
        //if ((walkState.iter + i) % (S + D) >= S) Ic(i,i) = 0;
        if ((walkState.footstepCounter*0 == 0) &&  ((walkState.iter + i) < S+D)) Ic(i,i) = 0;
    }

    // Construct the difference matrix (x_j - x_{j-1})
    Eigen::MatrixXd differenceMatrix = Eigen::MatrixXd::Identity(M,M);
    for (int i = 0; i < M-1; ++i) {
        differenceMatrix(i+1,i) = -1;
    }

    // Construct the stability constraint
    // **********************************

    double stabConstrMultiplier = (1-exp(-omega*timeStep)) / omega;
    for(int i = 0; i < N; ++i) {
        Aeq(0,i)     = stabConstrMultiplier * exp(-omega*timeStep*i);
        Aeq(1,N+M+i) = stabConstrMultiplier * exp(-omega*timeStep*i);
    }

    beq << state.com.pos(0) + state.com.vel(0)/omega - state.zmpPos(0),
           state.com.pos(1) + state.com.vel(1)/omega - state.zmpPos(1);

    // Construct the ZMP constraint
    // ****************************

    // Construct the A matrix of the ZMP constraint, by diagonalizing two of the same, then rotating
    Eigen::MatrixXd halfAZmp(N,N+M);
    halfAZmp << Ic*P, -Ic*Cc;
    AZmp.block(0,0,N,N+M) = halfAZmp;
    AZmp.block(N,N+M,N,N+M) = halfAZmp;

    // Construct the b vector of the ZMP constraint
    Eigen::VectorXd bZmpSizeTerm = Eigen::VectorXd::Zero(2*N);
    Eigen::VectorXd bZmpStateTerm = Eigen::VectorXd::Zero(2*N);

    bZmpSizeTerm << Ic*p*footConstraintSquareWidth/2, Ic*p*footConstraintSquareWidth/2;

    bZmpStateTerm << Ic*(-p*state.zmpPos(0)+currentFootstepZmp*supportFootPose(0)), Ic*(-p*state.zmpPos(1)+currentFootstepZmp*supportFootPose(1));

    bZmpMin = - bZmpSizeTerm + bZmpStateTerm;
    bZmpMax =   bZmpSizeTerm + bZmpStateTerm;

    // Construct the kinematic constraint
    // **********************************Orienta

    bool supportFootBoolean = walkState.supportFoot == Foot::RIGHT;
    
    // Construct the matrices that activate the left or right constraint
    Eigen::VectorXd pFr = Eigen::VectorXd::Zero(M);
    Eigen::VectorXd pFl = Eigen::VectorXd::Zero(M);
    Eigen::VectorXd pF = Eigen::VectorXd::Ones(M);
    for (int i = 0; i < M; ++i) {
        pFr(i) = (int)(supportFootBoolean + i) % 2;
    }
    pFl = pF - pFr;

    // Vector for the current footstep position
    Eigen::VectorXd currentFootstepKinematic = Eigen::VectorXd::Zero(M);
    currentFootstepKinematic(0) = 1;

    // Assemble the A matrix for the kinematic constraint, and rotate
    AFootsteps.block(0,N,M,M) = differenceMatrix;
    AFootsteps.block(M,2*N+M,M,M) = differenceMatrix;

    // Assemble the b vector for the kinematic constraint
    bFootstepsMin << -pF*deltaXMax + supportFootPose(0)*currentFootstepKinematic, -pFl*deltaYOut + pFr*deltaYIn  + supportFootPose(1)*currentFootstepKinematic;
    bFootstepsMax <<  pF*deltaXMax + supportFootPose(0)*currentFootstepKinematic, -pFl*deltaYIn  + pFr*deltaYOut + supportFootPose(1)*currentFootstepKinematic;

    // Construct the cost function
    // ***************************

    // Construct the H matrix, which is made of two of the same halfH block
    Eigen::MatrixXd halfH = Eigen::MatrixXd::Zero(N+M, N+M);
    halfH << qZd*Eigen::MatrixXd::Identity(N,N) + qZ*P.transpose()*P,
             -qZ*P.transpose()*Cc, -qZ*Cc.transpose()*P, qZ*Cc.transpose()*Cc + qF*Eigen::MatrixXd::Identity(M,M);
    costFunctionH.block(0,0,N+M,N+M) = halfH;
    costFunctionH.block(N+M,N+M,N+M,N+M) = halfH;

    // Contruct candidate footstep vectors
    Eigen::VectorXd xCandidateFootsteps(M), yCandidateFootsteps(M);
    for (int i = 0; i < M; i++) {
        xCandidateFootsteps(i) = plan->getFootstepPosition(walkState.footstepCounter + i + 1)(0);
        yCandidateFootsteps(i) = plan->getFootstepPosition(walkState.footstepCounter + i + 1)(1);
    }
    
    // Construct the F vector
    costFunctionF << qZ*P.transpose()*(p*state.zmpPos(0) - currentFootstepZmp*supportFootPose(0)),
		     -qZ*Cc.transpose()*(p*state.zmpPos(0) - currentFootstepZmp*supportFootPose(0)) - qF*xCandidateFootsteps,
		     qZ*P.transpose()*(p*state.zmpPos(1) - currentFootstepZmp*supportFootPose(1)),
		     -qZ*Cc.transpose()*(p*state.zmpPos(1) - currentFootstepZmp*supportFootPose(1)) - qF*yCandidateFootsteps;

    // Stack the constraint matrices
    // **************************************************************

    int nConstraints = AFootsteps.rows() + AZmp.rows();
    AConstraint.resize(nConstraints, 2*(N+M));
    bConstraintMin.resize(nConstraints);
    bConstraintMax.resize(nConstraints);

    AConstraint    << AFootsteps, AZmp;
    bConstraintMin << bFootstepsMin, bZmpMin;
    bConstraintMax << bFootstepsMax, bZmpMax;

    // Solve QP and update state
    // *************************

    ISMPC_qp_solver_ptr_->solve(costFunctionH, costFunctionF, Aeq, beq, AZmp, bZmpMin, bZmpMax);
    Eigen::VectorXd decisionVariables = ISMPC_qp_solver_ptr_->get_solution();

    // Split the QP solution in ZMP dot and footsteps
    Eigen::VectorXd zDotOptimalX = decisionVariables.head(N);
    Eigen::VectorXd zDotOptimalY = decisionVariables.segment(N+M,N);
    Eigen::VectorXd footstepsOptimalX = decisionVariables.segment(N,M);
    Eigen::VectorXd footstepsOptimalY = decisionVariables.segment(2*N+M,M);

    // Update the com-torso state based on the result of the QP
    double ch = cosh(omega*worldTimeStep);
    double sh = sinh(omega*worldTimeStep);

    Eigen::Matrix3d A_upd = Eigen::MatrixXd::Zero(3,3);
    Eigen::Vector3d B_upd = Eigen::VectorXd::Zero(3);
    A_upd<<ch,sh/omega,1-ch,omega*sh,ch,-omega*sh,0,0,1;
    B_upd<<worldTimeStep-sh/omega,1-ch,worldTimeStep;

    Eigen::Vector3d currentStateX = Eigen::Vector3d(state.com.pos(0), state.com.vel(0), state.zmpPos(0));
    Eigen::Vector3d currentStateY = Eigen::Vector3d(state.com.pos(1), state.com.vel(1), state.zmpPos(1));
    Eigen::Vector3d nextStateX = A_upd*currentStateX + B_upd*zDotOptimalX(0);
    Eigen::Vector3d nextStateY = A_upd*currentStateY + B_upd*zDotOptimalY(0);
    
    footstepPredicted << footstepsOptimalX(0),footstepsOptimalY(0),0.0,0.0;
    
    // Fill com_state with xy reference position (1st column) and velocity (2nd column)
    state.com.pos = Eigen::Vector3d(nextStateX(0), nextStateY(0), comTargetHeight);
    state.com.vel = Eigen::Vector3d(nextStateX(1), nextStateY(1), 0.0);
    state.zmpPos = Eigen::Vector3d(nextStateX(2), nextStateY(2), 0.0);
    
    // Generate feet trajectories
    
    Eigen::Vector3d currentFootstep = plan->getFootstepPosition(walkState.footstepCounter);
    Eigen::Vector3d targetFootstep = plan->getFootstepPosition(walkState.footstepCounter + 1);
    double kStep = 0.1;
    double kStepVert = 150;
    double kStepVert2 = 20;
    
    /*if (walkState.iter % (S+D) >= S || walkState.footstepCounter == 0) {
    	if (plan->isSupportFootLeft(walkState.footstepCounter)) {
    	    state.leftFoot.pos.head(2) = currentFootstep.head(2);
    	    state.leftFoot.pos(2) = groundHeight;
    	    state.leftFoot.vel.setZero();
    	    state.leftFoot.acc.setZero();
    	    state.rightFoot.pos.head(2) = targetFootstep.head(2);
    	    state.rightFoot.pos(2) = groundHeight;
    	    state.rightFoot.vel.setZero();
    	    state.rightFoot.acc.setZero();
    	} else {
    	    state.rightFoot.pos.head(2) = currentFootstep.head(2);
    	    state.rightFoot.pos(2) = groundHeight;
    	    state.rightFoot.vel.setZero();
    	    state.rightFoot.acc.setZero();
    	    state.leftFoot.pos.head(2) = targetFootstep.head(2);
    	    state.leftFoot.pos(2) = groundHeight;
    	    state.leftFoot.vel.setZero();
    	    state.leftFoot.acc.setZero();
    	}
    } else {
        if (plan->isSupportFootLeft(walkState.footstepCounter)) {
            state.leftFoot.pos.head(2) = currentFootstep.head(2);
            state.leftFoot.pos(2) = groundHeight;
    	    state.leftFoot.vel.setZero();
    	    state.leftFoot.acc.setZero();
            state.rightFoot.pos += timeStep * state.rightFoot.vel;
    	    state.rightFoot.vel += timeStep * state.rightFoot.acc;
    	    state.rightFoot.acc.head(2) = kStep * (targetFootstep.head(2) - state.rightFoot.pos.head(2));
    	    if(walkState.iter % (S+D) < S/2) {
    	        state.rightFoot.acc(2) = kStepVert * (groundHeight + stepHeight - state.rightFoot.pos(2)) - kStepVert2 * state.rightFoot.vel(2);
    	    } else {
    	        state.rightFoot.acc(2) = kStepVert * (groundHeight - state.rightFoot.pos(2)) - kStepVert2 * state.rightFoot.vel(2);
    	    }
    	} else {
    	    state.rightFoot.pos.head(2) = currentFootstep.head(2);
    	    state.rightFoot.pos(2) = groundHeight;
    	    state.rightFoot.vel.setZero();
    	    state.rightFoot.acc.setZero();
    	    state.leftFoot.pos += timeStep * state.leftFoot.vel;
    	    state.leftFoot.vel += timeStep * state.leftFoot.acc;
    	    state.leftFoot.acc.head(2) = kStep * (targetFootstep.head(2) - state.leftFoot.pos.head(2));
    	    if(walkState.iter % (S+D) < S/2) {
    	        state.leftFoot.acc(2) = kStepVert * (groundHeight + stepHeight - state.leftFoot.pos(2)) - kStepVert2 * state.leftFoot.vel(2);
    	    } else {
    	        state.leftFoot.acc(2) = kStepVert * (groundHeight - state.leftFoot.pos(2)) - kStepVert2 * state.leftFoot.vel(2);
    	    }
    	}
    }*/
    
    Eigen::VectorXd previousFootstepPos, currentFootstepPos, nextFootstepPos, movingConstraint;

    if (walkState.footstepCounter == 0) previousFootstepPos = plan->getFootstepPosition(walkState.footstepCounter + 1);
    else previousFootstepPos = plan->getFootstepPosition(walkState.footstepCounter - 1);

    currentFootstepPos = plan->getFootstepPosition(walkState.footstepCounter);
    nextFootstepPos = plan->getFootstepPosition(walkState.footstepCounter + 1);
    currentFootstepPos(2) = groundHeight;
    nextFootstepPos(2) = groundHeight;
    previousFootstepPos(2) = groundHeight;
    
    double actualStepHeight = walkState.footstepCounter == 0 ? 0 : stepHeight;

    if (walkState.iter % (S+D) < S) { // we are in single support

        double stepCompletion = (double)(walkState.iter % (S+D)) / (double)S;

        if (plan->isSupportFootLeft(walkState.footstepCounter)) {
            // move right foot
            state.leftFoot.pos = currentFootstepPos;
            state.rightFoot.pos = previousFootstepPos + (nextFootstepPos - previousFootstepPos) * cubic(stepCompletion);
                
            state.leftFoot.vel = Eigen::Vector3d::Zero();
            state.rightFoot.vel = (nextFootstepPos - previousFootstepPos) * cubic_dot(stepCompletion) / singleSupportDuration;
                
            state.leftFoot.acc = Eigen::Vector3d::Zero();
            state.rightFoot.acc = (nextFootstepPos - previousFootstepPos) * cubic_ddot(stepCompletion) / singleSupportDuration / singleSupportDuration;
                
            if (stepCompletion <= 0.5) {
		state.rightFoot.pos(2) += cubic(stepCompletion*2) * actualStepHeight;
		state.rightFoot.vel(2) += cubic_dot(stepCompletion*2) * actualStepHeight * 2 / singleSupportDuration;
		state.rightFoot.acc(2) += cubic_ddot(stepCompletion*2) * actualStepHeight * 4 / singleSupportDuration / singleSupportDuration;
            } else {
		state.rightFoot.pos(2) += actualStepHeight - cubic((stepCompletion-0.5)*2) * actualStepHeight;
		state.rightFoot.vel(2) += - cubic_dot((stepCompletion-0.5)*2) * actualStepHeight * 2 / singleSupportDuration;
		state.rightFoot.acc(2) += - cubic_ddot((stepCompletion-0.5)*2) * actualStepHeight * 4 / singleSupportDuration / singleSupportDuration;
            }
        } else {
                // move left foot
                state.rightFoot.pos = currentFootstepPos;
                state.leftFoot.pos = previousFootstepPos + (nextFootstepPos - previousFootstepPos) * cubic(stepCompletion);
                
                state.rightFoot.vel = Eigen::Vector3d::Zero();
                state.leftFoot.vel= (nextFootstepPos - previousFootstepPos) * cubic_dot(stepCompletion) / singleSupportDuration;
                
                state.rightFoot.acc = Eigen::Vector3d::Zero();
                state.leftFoot.acc= (nextFootstepPos - previousFootstepPos) * cubic_ddot(stepCompletion) / singleSupportDuration / singleSupportDuration;
                
                if (stepCompletion <= 0.5) {
		    state.leftFoot.pos(2) += cubic(stepCompletion*2) * actualStepHeight;
		    state.leftFoot.vel(2) += cubic_dot(stepCompletion*2) * actualStepHeight * 2 / singleSupportDuration;
		    state.leftFoot.acc(2) += cubic_ddot(stepCompletion*2) * actualStepHeight * 4 / singleSupportDuration / singleSupportDuration;
		} else {
		    state.leftFoot.pos(2) += actualStepHeight - cubic((stepCompletion-0.5)*2) * actualStepHeight;
		    state.leftFoot.vel(2) += - cubic_dot((stepCompletion-0.5)*2) * actualStepHeight * 2 / singleSupportDuration;
		    state.leftFoot.acc(2) += - cubic_ddot((stepCompletion-0.5)*2) * actualStepHeight * 4 / singleSupportDuration / singleSupportDuration;
		}
            }
        } else { // we are in double support
            if (plan->isSupportFootLeft(walkState.footstepCounter)) { 
                state.leftFoot.pos = currentFootstepPos;
                state.rightFoot.pos = nextFootstepPos;
            } else {
                state.leftFoot.pos = nextFootstepPos;
                state.rightFoot.pos = currentFootstepPos;
            }
            state.leftFoot.vel = Eigen::Vector3d::Zero();
            state.rightFoot.vel = Eigen::Vector3d::Zero();
            
            state.leftFoot.acc = Eigen::Vector3d::Zero();
            state.rightFoot.acc = Eigen::Vector3d::Zero();
        }

    return state;
}
