#include "Controller.hpp"

Controller::Controller(dart::dynamics::SkeletonPtr _robot, dart::simulation::WorldPtr _world)
	: mRobot(_robot), mWorld(_world)
{
    setInitialConfiguration();

    // Some useful pointers to robot limbs
    mLeftFoot = mRobot->getBodyNode("l_sole");
    mRightFoot = mRobot->getBodyNode("r_sole");
    mBase = mRobot->getBodyNode("base_link");
    mTorso = mRobot->getBodyNode("torso");

    // Initialize walk state
    walkState.iter = 0;
    walkState.footstepCounter = 0;
    walkState.supportFoot = Foot::RIGHT;

    // Retrieve current state

    current = getCurrentRobotState();

    // Initialize desired state with reasonable values

    desired.com.pos = current.com.pos;
    desired.com.vel = Eigen::Vector3d::Zero();
    desired.com.acc = Eigen::Vector3d::Zero();
    desired.com.ang_pos = Eigen::Vector3d::Zero();
    desired.com.ang_vel = Eigen::Vector3d::Zero();
    desired.com.ang_acc = Eigen::Vector3d::Zero();

    desired.zmpPos = Eigen::Vector3d(current.com.pos(0), current.com.pos(1),0.0);

    desired.leftFoot.pos = current.leftFoot.pos;
    desired.leftFoot.vel = Eigen::Vector3d::Zero();
    desired.leftFoot.acc = Eigen::Vector3d::Zero();
    desired.leftFoot.ang_pos = Eigen::Vector3d::Zero();
    desired.leftFoot.ang_vel = Eigen::Vector3d::Zero();
    desired.leftFoot.ang_acc = Eigen::Vector3d::Zero();

    desired.rightFoot.pos = current.rightFoot.pos;
    desired.rightFoot.vel = Eigen::Vector3d::Zero();
    desired.rightFoot.acc = Eigen::Vector3d::Zero();
    desired.rightFoot.ang_pos = Eigen::Vector3d::Zero();
    desired.rightFoot.ang_vel = Eigen::Vector3d::Zero();
    desired.rightFoot.ang_acc = Eigen::Vector3d::Zero();

    initial = desired;
    
    // Generate reference velocity
    std::vector<Vref> vrefSequence;

    for (int i = 0; i < 5000; i++) {
        if (i < 100) vrefSequence.push_back(Vref(0.0, 0.0, 0.0));
        else vrefSequence.push_back(Vref(0.3, 0.0, 0.0));
        //vrefSequence.push_back(Vref(0.0, 0.0, 0.0));
    }
    
    // Plan footsteps
    bool firstSupportFootIsLeft = false;
    Eigen::VectorXd leftFootPose(6), rightFootPose(6);
    leftFootPose << getRPY(mLeftFoot->getTransform().rotation()), mLeftFoot->getCOM();
    rightFootPose << getRPY(mRightFoot->getTransform().rotation()), mRightFoot->getCOM();
    
    footstepPlan = new FootstepPlan;
    footstepPlan->plan(vrefSequence, leftFootPose, rightFootPose, firstSupportFootIsLeft);
    
    // Compute referefence trajectory
    // this computes reference trajectory according to footstep plan
    //ref = computeReferenceTrajectory(footstepPlan, initial, 2000);
    // this fill the reference trajectory with the initial state
    for (int i = 0; i < 2000; i++) {
        ref.push_back(initial);
    }
    
    ismpc = std::make_unique<ISMPC>(footstepPlan, initial, mRobot);

    // Create file loggers
    logList.push_back(new Logger("desired.comPos", &desired.com.pos));
    logList.push_back(new Logger("desired.comVel", &desired.com.vel));
    logList.push_back(new Logger("desired.comAcc", &desired.com.acc));
    logList.push_back(new Logger("desired.zmpPos", &desired.zmpPos));

    logList.push_back(new Logger("current.comPos", &current.com.pos));
    logList.push_back(new Logger("current.comVel", &current.com.vel));
    logList.push_back(new Logger("current.comAcc", &current.com.acc));
    logList.push_back(new Logger("current.zmpPos", &current.zmpPos));
    
    logList.push_back(new Logger("desired.leftFoot.pos", &desired.leftFoot.pos));
    logList.push_back(new Logger("desired.rightFoot.pos", &desired.rightFoot.pos));
    logList.push_back(new Logger("current.leftFoot.pos", &current.leftFoot.pos));
    logList.push_back(new Logger("current.rightFoot.pos", &current.rightFoot.pos));
    
    logList.push_back(new Logger("desired.leftFoot.vel", &desired.leftFoot.vel));
    logList.push_back(new Logger("desired.rightFoot.vel", &desired.rightFoot.vel));
    logList.push_back(new Logger("current.leftFoot.vel", &current.leftFoot.vel));
    logList.push_back(new Logger("current.rightFoot.vel", &current.rightFoot.vel));
    
    logList.push_back(new Logger("desired.leftFoot.acc", &desired.leftFoot.acc));
    logList.push_back(new Logger("desired.rightFoot.acc", &desired.rightFoot.acc));
    logList.push_back(new Logger("current.leftFoot.acc", &current.leftFoot.acc));
    logList.push_back(new Logger("current.rightFoot.acc", &current.rightFoot.acc));
    
    logList.push_back(new Logger("desired.com.ang", &desired.com.ang_pos));
    logList.push_back(new Logger("current.com.ang", &current.com.ang_pos));
}

Controller::~Controller() {}

void Controller::update() {
    walkState.simulationTime = mWorld->getSimFrames();
    
    
    
    // This adds a push to the robot
    //if (walkState.simulationTime>=410 && walkState.simulationTime<=420) mTorso->addExtForce(Eigen::Vector3d(0,70,0));

    // Retrieve current and desired state
    current = getCurrentRobotState();
    //desired = ref.at(walkState.iter);
    //walkState.supportFoot = footstepPlan->getFootstepIndexAtTime(walkState.iter) % 2 == 0 ? walkState.supportFoot = Foot::RIGHT : walkState.supportFoot = Foot::LEFT;
    walkState.supportFoot = walkState.footstepCounter % 2 == 0 ? walkState.supportFoot = Foot::RIGHT : walkState.supportFoot = Foot::LEFT;

    // Store the results in files (for plotting)
    for (int i = 0; i < logList.size(); ++i) {
        logList.at(i)->log();
    }

    desired = ismpc->MPC(walkState, current);

    // Compute inverse kinematics
    Eigen::VectorXd qDot =  getJointVelocities(desired, current, walkState);

    // Set the acceleration of each joint
    for (int i = 0; i < 50; ++i) {
        mRobot->setCommand(i+6,qDot(i));
    }

    // right arm
    mRobot->setPosition(mRobot->getDof("R_SHOULDER_P")->getIndexInSkeleton(), (4)*M_PI/180 );
    mRobot->setPosition(mRobot->getDof("R_SHOULDER_R")->getIndexInSkeleton(), -8*M_PI/180  );
    mRobot->setPosition(mRobot->getDof("R_SHOULDER_Y")->getIndexInSkeleton(), 0 );
    mRobot->setPosition(mRobot->getDof("R_ELBOW_P")->getIndexInSkeleton(), -25*M_PI/180 );

    // left arm
    mRobot->setPosition(mRobot->getDof("L_SHOULDER_P")->getIndexInSkeleton(), (4)*M_PI/180  );
    mRobot->setPosition(mRobot->getDof("L_SHOULDER_R")->getIndexInSkeleton(), 8*M_PI/180  );
    mRobot->setPosition(mRobot->getDof("L_SHOULDER_Y")->getIndexInSkeleton(), 0 );
    mRobot->setPosition(mRobot->getDof("L_ELBOW_P")->getIndexInSkeleton(), -25*M_PI/180 );

    // Arm swing
    //mRobot->setPosition(mRobot->getDof("R_SHOULDER_P")->getIndexInSkeleton(), (4+5*sin(2*M_PI*0.01*(mWorld->getSimFrames())))*M_PI/180);
    //mRobot->setPosition(mRobot->getDof("L_SHOULDER_P")->getIndexInSkeleton(), (4-5*sin(2*M_PI*0.01*(mWorld->getSimFrames())))*M_PI/180);
    /*double leftShoulderDes = mRobot->getPosition(mRobot->getDof("R_HIP_P")->getIndexInSkeleton()) + M_PI/4;
    double rightShoulderDes = mRobot->getPosition(mRobot->getDof("L_HIP_P")->getIndexInSkeleton()) + M_PI/4;
    double leftShoulderCurr = mRobot->getPosition(mRobot->getDof("L_SHOULDER_P")->getIndexInSkeleton());
    double rightShoulderCurr = mRobot->getPosition(mRobot->getDof("R_SHOULDER_P")->getIndexInSkeleton());

    double kArms = 0.1;
    mRobot->setVelocity(mRobot->getDof("L_SHOULDER_P")->getIndexInSkeleton(), - kArms * (leftShoulderCurr - leftShoulderCurr));
    mRobot->setVelocity(mRobot->getDof("R_SHOULDER_P")->getIndexInSkeleton(), - kArms * (rightShoulderCurr - rightShoulderCurr));*/
    
    // Draw on the screen
    draw();

    // Update the iteration counters
    ++walkState.iter;
    walkState.footstepCounter = walkState.iter / (S + D);
}

Eigen::VectorXd Controller::getJointVelocities(State desired, State current, WalkState walkState) {

    // WEIGHTS
    double left_weight = 0.1;
    double right_weight = 0.1;
    double CoM_weight = 1;
    double qdot_weight = 1e-6;
    double base_weight = 1;
    
    int n_dof_ = 56;
    double CoM_gains = 0.5;
    double left_gains = 1;
    double right_gains = 1;
    double base_gains = 0.5;
    
    // Jacobians
    Eigen::MatrixXd Jacobian_leftFoot, Jacobian_rightFoot, Jacobian_com, Jacobian_torso_angle;
    Jacobian_leftFoot = mRobot->getJacobian(mLeftFoot);
    Jacobian_rightFoot = mRobot->getJacobian(mRightFoot);
    Jacobian_com.resize(6,56);
    Jacobian_com << mRobot->getAngularJacobian(mTorso), mRobot->getCOMLinearJacobian();
    
    Eigen::MatrixXd Jacobian_base = mRobot->getAngularJacobian(mBase);

    // Generate cartesian error vector   
    Eigen::VectorXd left_errorVector = desired.getLeftFootPose() - current.getLeftFootPose();
    left_errorVector.segment(0,3) = angleSignedDistance(desired.getLeftFootPose().segment(0,3), current.getLeftFootPose().segment(0,3));
    Eigen::VectorXd right_errorVector = desired.getRightFootPose() - current.getRightFootPose();
    right_errorVector.segment(0,3) = angleSignedDistance(desired.getRightFootPose().segment(0,3), current.getRightFootPose().segment(0,3));
    Eigen::VectorXd com_errorVector = desired.getComPose() - current.getComPose();
    com_errorVector.segment(0,3) = angleSignedDistance(desired.getComPose().segment(0,3), current.getComPose().segment(0,3));

    // Cost function
    Eigen::MatrixXd costFunctionH = qdot_weight * Eigen::MatrixXd::Identity(n_dof_, n_dof_);
    Eigen::VectorXd costFunctionF = Eigen::VectorXd::Zero(n_dof_);

    costFunctionH += CoM_weight * Jacobian_com.transpose()*Jacobian_com;
    costFunctionF += - CoM_weight * Jacobian_com.transpose() * (desired.getComVelocity() + CoM_gains*com_errorVector);

    costFunctionH += left_weight * Jacobian_leftFoot.transpose()*Jacobian_leftFoot;
    costFunctionF += - left_weight * Jacobian_leftFoot.transpose() * (desired.getLeftFootVelocity() + left_gains*left_errorVector);
  
    costFunctionH += right_weight * Jacobian_rightFoot.transpose()*Jacobian_rightFoot;
    costFunctionF += - right_weight * Jacobian_rightFoot.transpose() * (desired.getRightFootVelocity() + right_gains*right_errorVector);
    
    costFunctionH += base_weight * Jacobian_base.transpose()*Jacobian_base;
    costFunctionF += - base_weight * Jacobian_base.transpose() * (- base_gains*getRPY(mBase->getTransform().rotation()));

    // input constraints
    Eigen::MatrixXd jointLimConstrMatrix = 0*Eigen::MatrixXd::Identity(n_dof_, n_dof_);
    Eigen::VectorXd jointLimUpperBound = 0*10 * Eigen::VectorXd::Ones(n_dof_);
    Eigen::VectorXd jointLimLowerBound = -0*10 * Eigen::VectorXd::Ones(n_dof_);
    
    // dummy equality constraint
    Eigen::MatrixXd A_dummy = Eigen::MatrixXd::Zero(6,n_dof_);
    Eigen::VectorXd b_dummy = Eigen::VectorXd::Zero(6);
  
  
    //if (desired.getLeftFootVelocity().norm() < 1e-6 && desired.getRightFootVelocity().norm() > 1e-6) A_dummy = Jacobian_leftFoot;
    //if (desired.getRightFootVelocity().norm() < 1e-6 && desired.getLeftFootVelocity().norm() > 1e-6) A_dummy = Jacobian_rightFoot;

    if (walkState.supportFoot == Foot::LEFT) A_dummy = Jacobian_leftFoot;
    else				     A_dummy = Jacobian_rightFoot;

    std::shared_ptr<labrob::qpsolvers::QPSolverEigenWrapper<double>> IK_qp_solver_ptr_ = std::make_shared<labrob::qpsolvers::QPSolverEigenWrapper<double>>(
        std::make_shared<labrob::qpsolvers::HPIPMQPSolver>(n_dof_, 6, n_dof_));

    IK_qp_solver_ptr_->solve(
        costFunctionH,
        costFunctionF,
        A_dummy,
        b_dummy,
        jointLimConstrMatrix,
        jointLimLowerBound,
        jointLimUpperBound
    );
    Eigen::VectorXd qDot_des = (IK_qp_solver_ptr_->get_solution());
    
    return qDot_des.tail(50);
}

State Controller::getCurrentRobotState()
{
    // Retrieve current state in the world frame
    State state;

    state.com.pos = mRobot->getCOM();
    state.com.vel = mRobot->getCOMLinearVelocity();
    state.com.acc = mRobot->getCOMLinearAcceleration();
    state.com.ang_pos = getRPY(mTorso->getTransform().rotation());
    state.com.ang_vel = mTorso->getCOMSpatialVelocity().head(3);
    state.com.ang_acc = mTorso->getCOMSpatialAcceleration().head(3);

    state.zmpPos = state.com.pos - state.com.acc / (omega*omega); //getZmpFromExternalForces(); //

    state.leftFoot.pos = mLeftFoot->getCOM();
    state.leftFoot.vel = mLeftFoot->getCOMLinearVelocity();
    state.leftFoot.acc = mLeftFoot->getCOMLinearAcceleration();
    state.leftFoot.ang_pos = getRPY(mLeftFoot->getTransform().rotation());
    state.leftFoot.ang_vel = mLeftFoot->getCOMSpatialVelocity().head(3);
    state.leftFoot.ang_acc = mLeftFoot->getCOMSpatialAcceleration().head(3);

    state.rightFoot.pos = mRightFoot->getCOM();
    state.rightFoot.vel = mRightFoot->getCOMLinearVelocity();
    state.rightFoot.acc = mRightFoot->getCOMLinearAcceleration();
    state.rightFoot.ang_pos = getRPY(mRightFoot->getTransform().rotation());
    state.rightFoot.ang_vel = mRightFoot->getCOMSpatialVelocity().head(3);
    state.rightFoot.ang_acc = mRightFoot->getCOMSpatialAcceleration().head(3);
    
    return state;
}

Eigen::Vector3d Controller::getZmpFromExternalForces()
{
    Eigen::Vector3d zmp_v;
    bool left_contact = false;
    bool right_contact = false;

    Eigen::Vector3d left_cop;
    if(abs(mLeftFoot->getConstraintImpulse()[5]) > 0.01){
        left_cop << -mLeftFoot->getConstraintImpulse()(1)/mLeftFoot->getConstraintImpulse()(5), mLeftFoot->getConstraintImpulse()(0)/mLeftFoot->getConstraintImpulse()(5), 0.0;
        Eigen::Matrix3d iRotation = mLeftFoot->getWorldTransform().rotation();
        Eigen::Vector3d iTransl   = mLeftFoot->getWorldTransform().translation();
        left_cop = iTransl + iRotation*left_cop;
        left_contact = true;
    }

    Eigen::Vector3d right_cop;
    if(abs(mRightFoot->getConstraintImpulse()[5]) > 0.01){
        right_cop << -mRightFoot->getConstraintImpulse()(1)/mRightFoot->getConstraintImpulse()(5), mRightFoot->getConstraintImpulse()(0)/mRightFoot->getConstraintImpulse()(5), 0.0;
        Eigen::Matrix3d iRotation = mRightFoot->getWorldTransform().rotation();
        Eigen::Vector3d iTransl   = mRightFoot->getWorldTransform().translation();
        right_cop = iTransl + iRotation*right_cop;
        right_contact = true;
    }

    if(left_contact && right_contact){
        zmp_v << (left_cop(0)*mLeftFoot->getConstraintImpulse()[5] + right_cop(0)*mRightFoot->getConstraintImpulse()[5])/(mLeftFoot->getConstraintImpulse()[5] + mRightFoot->getConstraintImpulse()[5]),
                 (left_cop(1)*mLeftFoot->getConstraintImpulse()[5] + right_cop(1)*mRightFoot->getConstraintImpulse()[5])/(mLeftFoot->getConstraintImpulse()[5] + mRightFoot->getConstraintImpulse()[5]),
		 0.0;
    }else if(left_contact){
        zmp_v << left_cop(0), left_cop(1), 0.0;
    }else if(right_contact){
        zmp_v << right_cop(0), right_cop(1), 0.0;
    }else{
        // No contact detected
        zmp_v << 0.0, 0.0, 0.0;
    }

    return zmp_v;
}

void Controller::draw() {
    std::shared_ptr<dart::dynamics::BoxShape> box;
    box = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(footConstraintSquareWidth, footConstraintSquareWidth, 0.01));
    //dart::dynamics::BoxShape::Properties box_properties;
    //box->setPositions(
    //mTorso->createShapeNodeWith<VisualAspect>(mArrow);
}

void Controller::setInitialConfiguration() {
    initialConfiguration = mRobot->getPositions();

    // floating base
    mRobot->setPosition(0, 0.0 );
    mRobot->setPosition(1, -0*M_PI/180.0);
    mRobot->setPosition(2, 0.0 );
    mRobot->setPosition(3, 0.035502257 -0.0);
    mRobot->setPosition(4, -0.0);
    mRobot->setPosition(5, 0.751 +0.00138 - 0.000005);

    // right leg
    mRobot->setPosition(mRobot->getDof("R_HIP_Y")->getIndexInSkeleton(), 0.0 );
    mRobot->setPosition(mRobot->getDof("R_HIP_R")->getIndexInSkeleton(), -3*M_PI/180 );
    mRobot->setPosition(mRobot->getDof("R_HIP_P")->getIndexInSkeleton(), -25*M_PI/180 );
    mRobot->setPosition(mRobot->getDof("R_KNEE_P")->getIndexInSkeleton(), 50*M_PI/180 );
    mRobot->setPosition(mRobot->getDof("R_ANKLE_P")->getIndexInSkeleton(), -26*M_PI/180+0.0175 );
    mRobot->setPosition(mRobot->getDof("R_ANKLE_R")->getIndexInSkeleton(), 4*M_PI/180-0.01745);

    // left leg
    mRobot->setPosition(mRobot->getDof("L_HIP_Y")->getIndexInSkeleton(), 0.0 );
    mRobot->setPosition(mRobot->getDof("L_HIP_R")->getIndexInSkeleton(), 3*M_PI/180 );
    mRobot->setPosition(mRobot->getDof("L_HIP_P")->getIndexInSkeleton(), -25*M_PI/180 );
    mRobot->setPosition(mRobot->getDof("L_KNEE_P")->getIndexInSkeleton(), 50*M_PI/180 );
    mRobot->setPosition(mRobot->getDof("L_ANKLE_P")->getIndexInSkeleton(), -26*M_PI/180+0.0175 );
    mRobot->setPosition(mRobot->getDof("L_ANKLE_R")->getIndexInSkeleton(), -4*M_PI/180+0.01745);

    // right arm
    mRobot->setPosition(mRobot->getDof("R_SHOULDER_P")->getIndexInSkeleton(), (4)*M_PI/180 );
    mRobot->setPosition(mRobot->getDof("R_SHOULDER_R")->getIndexInSkeleton(), -8*M_PI/180  );
    mRobot->setPosition(mRobot->getDof("R_SHOULDER_Y")->getIndexInSkeleton(), 0 );
    mRobot->setPosition(mRobot->getDof("R_ELBOW_P")->getIndexInSkeleton(), -25*M_PI/180 );

    // left arm
    mRobot->setPosition(mRobot->getDof("L_SHOULDER_P")->getIndexInSkeleton(), (4)*M_PI/180  );
    mRobot->setPosition(mRobot->getDof("L_SHOULDER_R")->getIndexInSkeleton(), 8*M_PI/180  );
    mRobot->setPosition(mRobot->getDof("L_SHOULDER_Y")->getIndexInSkeleton(), 0 );
    mRobot->setPosition(mRobot->getDof("L_ELBOW_P")->getIndexInSkeleton(), -25*M_PI/180 );
  
    initialConfiguration = mRobot->getPositions();
}


