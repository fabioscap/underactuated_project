#include "Controller.hpp"
#include "task.hpp"

#define COLOR_RED (Eigen::Vector3d()<<1.0,0.0,0.0).finished()
#define COLOR_PURPLE (Eigen::Vector3d()<<0.6,0.44,0.86).finished()
#define COLOR_GREEN (Eigen::Vector3d()<<0.0,1.0,0.0).finished()

Controller::Controller(dart::dynamics::SkeletonPtr _robot, dart::simulation::WorldPtr _world)
	: mRobot(_robot), mWorld(_world), visualize(true) {

  setInitialConfiguration();
  y_file.open("Y.txt");
  com_file.open("COM.txt");
  com_des_file.open("COM_DES.txt");
  ZMP_sol_file.open("ZMP_SOL.txt");
  ZMP_file.open("ZMP.txt");

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
  initial = getCurrentRobotState();
  initial_com = current.com.pos;
  // Initialize desired state with reasonable values

  desired = getDesiredRobotState(0);
    
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
  
  //footstepPlan = new FootstepPlan;
  //footstepPlan->plan(vrefSequence, leftFootPose, rightFootPose, firstSupportFootIsLeft);
  
  // Compute referefence trajectory
  // this computes reference trajectory according to footstep plan
  //ref = computeReferenceTrajectory(footstepPlan, initial, 2000);
  // this fill the reference trajectory with the initial state
  for (int i = 0; i < 2000; i++) {
      ref.push_back(initial);
  }
  
  // ismpc = std::make_unique<ISMPC>(footstepPlan, initial, mRobot);

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
  
  logList.push_back(new Logger("desired.leftFoot.ang_pos", &desired.leftFoot.ang_pos));
  logList.push_back(new Logger("desired.leftFoot.ang_vel", &desired.leftFoot.ang_vel));
  logList.push_back(new Logger("desired.leftFoot.ang_acc", &desired.leftFoot.ang_acc));

  
  // create a visualizer for the COM, ZMP, ...
  /* OK but gravity 
  dart::dynamics::SkeletonPtr visualizer = dart::dynamics::Skeleton::create("visualizer");

  dart::dynamics::FreeJoint::Properties joint_properties = dart::dynamics::FreeJoint::Properties();
  joint_properties.mName = "COM_joint";


  dart::dynamics::BodyNodePtr COM_viz = visualizer->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(nullptr,
                                                                                                          joint_properties, 
                                                                                                          dart::dynamics::BodyNode::AspectProperties("COM_viz")).second;
  COM_viz->createShapeNodeWith<dart::dynamics::VisualAspect>(std::make_shared<dart::dynamics::EllipsoidShape>(3.0 * Eigen::Vector3d::Ones()));
  //COM_viz->setColor(Eigen::Vector4d::Ones());
  mWorld->addSkeleton(visualizer);
  */
  
  COM_viz = registerBall("COM_viz", COLOR_RED);
  COM_des_viz = registerBall("COM_des_viz", COLOR_PURPLE);
  ZMP_viz = registerBall("ZMP_viz", COLOR_GREEN);
}

Controller::~Controller() {}

void Controller::update() {
  walkState.simulationTime = mWorld->getSimFrames();
  
  // This adds a push to the robot
  if (walkState.simulationTime>=410 && walkState.simulationTime<=420) mTorso->addExtForce(Eigen::Vector3d(0,100,0));

  // Retrieve current and desired state
  current = getCurrentRobotState();
  //desired = ref.at(walkState.iter);
  desired = getDesiredRobotState(walkState.simulationTime);
  //walkState.supportFoot = footstepPlan->getFootstepIndexAtTime(walkState.iter) % 2 == 0 ? walkState.supportFoot = Foot::RIGHT : walkState.supportFoot = Foot::LEFT;
  //walkState.supportFoot = walkState.footstepCounter % 2 == 0 ? walkState.supportFoot = Foot::RIGHT : walkState.supportFoot = Foot::LEFT;

  // Store the results in files (for plotting)
  for (int i = 0; i < logList.size(); ++i) {
      logList.at(i)->log();
  }

  //desired = ismpc->MPC(walkState, current);

  Eigen::VectorXd tau =  getJointTorques(desired, current);

  // Set the torque of each joint
  for (int i = 0; i < 50; ++i) {
      mRobot->setCommand(i+6,tau(i));
  }

  // Draw on the screen
  draw();

  // Update the iteration counters
  //++walkState.iter;
  //walkState.footstepCounter = walkState.iter / (S + D);
}

Eigen::VectorXd Controller::getJointTorques(State& desired,State& current) {
  // com
  double m = mRobot->getMass();

  Matrixd<56, 56> M = mRobot->getMassMatrix(); // 56x56
  Matrixd<56,1> q_dot = mRobot->getVelocities();

  // split matrix into M_u (joints 50x56) and M_l (COM 6x56)
  // first 6 components are realtive to floating base
  Matrixd<6, 56> M_l = M.block<6,56>(0,0);
  // last 50 components are relative to joints
  Matrixd<50,56> M_u = M.block<50,56>(6,0);

  Matrixd<56, 1> N = mRobot->getCoriolisAndGravityForces();

  Matrixd<6, 1> N_l = N.block<6,1>(0,0);
  Matrixd<50, 1> N_u = N.block<50,1>(6,0);

  // piedi's Jacobian 
  Matrixd<6, 56> J_leftFoot = mRobot->getJacobian(mLeftFoot);
  Matrixd<6, 56> J_rightFoot = mRobot->getJacobian(mRightFoot);
  Matrixd<6, 56> Jdot_leftFoot = mRobot->getJacobianClassicDeriv(mLeftFoot);
  Matrixd<6, 56> Jdot_rightFoot = mRobot->getJacobianClassicDeriv(mRightFoot);

  // angular then linear
  Matrixd<6, 1> leftFoot_pose = current.getLeftFootPose();
  Matrixd<6, 1> rightFoot_pose = current.getRightFootPose();
  Matrixd<6, 1> leftFoot_velocity = current.getLeftFootVelocity();
  Matrixd<6, 1> rightFoot_velocity = current.getRightFootVelocity();
  
  Matrixd<12, 1> feet_velocities;
  feet_velocities << leftFoot_velocity, rightFoot_velocity ;

  Matrixd<12,56> J_contact;
  J_contact << J_leftFoot, J_rightFoot;
  Matrixd<12,56> Jdot_contact;
  Jdot_contact << Jdot_leftFoot, Jdot_rightFoot;

  Matrixd<12,6> J_contact_l;
  J_contact_l << J_leftFoot.block<6,6>(0,0), J_rightFoot.block<6,6>(0,0);

  Matrixd<12,50> J_contact_u;
  J_contact_u << J_leftFoot.block<6,50>(0,6), J_rightFoot.block<6,50>(0,6);

  hrc::HierarchicalSolver solver = hrc::HierarchicalSolver(56+12);

  // highest priority task: Newton dynamics
  Eigen::Matrix<double,6,56+12> B1;
  B1 << M_l, -J_contact_l.transpose();

  // the feet do not move
  // add left_errorVector_vel / timeStep;
  Matrixd<12,56+12> B2;
  B2 << J_contact, Matrixd<12,12>::Zero();
  Matrixd<12, 1> b2 = Jdot_contact*q_dot - (1/(100*timeStep))*(Matrixd<12,1>::Zero() - feet_velocities);


  // COM frame is aligned as world frame
  Matrixd<6,6> Phi1 = Matrixd<6,6>::Zero();
  auto R01 = mBase->getTransform().rotation();
  Phi1.block<3,3>(0,0) = R01;
  Phi1.block<3,3>(3,3) = R01;

  Matrixd<6,6> X1Gt = Matrixd<6,6>::Zero();
  X1Gt.block<3,3>(0,0) = R01;
  X1Gt.block<3,3>(3,3) = R01;
  X1Gt.block<3,3>(0,3) = -R01*skew(mRobot->getCOM(mBase));


  Matrixd<6,56> Ag    = X1Gt*Phi1*M_l;
  //print_shape("coriolis", mRobot->getCoriolisForces().block<6,1>(0,0));
  Matrixd<6,1> Agdqd = X1Gt*Phi1*((mRobot->getCoriolisForces()).block<6,1>(0,0));

  double K_x = 10;
  double K_y = 10;
  double K_z = 10;
  Eigen::Matrix6d K_p = (Eigen::Vector6d() << 0,0,0,K_x, K_y, K_z).finished().asDiagonal(); 
  double K_d=10;

  Matrixd<6,56+12> B3 ;
  B3<< Ag, Matrixd<6,12>::Zero();

  Matrixd<6,1>   b3 = Agdqd -K_p*((Eigen::Vector6d()<<Eigen::Vector3d::Zero(), desired.com.pos - current.com.pos).finished())
                            -K_d*((Eigen::Vector6d::Zero()-Ag*q_dot)); // - FFW
    
  // test for inequality constraints
  // accel of COM along z axis is bounded
  /*
  double a_z_max = 8.0;

  Matrixd<1,56+12> C1 = Matrixd<1,56+12>::Zero();
  Matrixd<1, 1>    l1;
  Matrixd<1, 1>    u1;
  C1.block(0,0,1,56) = Ag.row(5);
  l1 << -Agdqd(5,0) -a_z_max;
  u1 << -Agdqd(5,0) +a_z_max;

  solver.add_ineq_contstr(C1, l1, u1, 1);
  //*/

  // ZMP constraints
  double dx = 0.08; double dy = 0.1; double high_val = 1000;
  auto leftFootTransform = mLeftFoot->getWorldTransform();
  auto rightFootTransform = mRightFoot->getWorldTransform();
  Eigen::Matrix3d iRotation_r = rightFootTransform.rotation();
  Eigen::Vector3d iTransl_r   = rightFootTransform.translation();
  Eigen::Matrix3d iRotation_l = leftFootTransform.rotation();
  Eigen::Vector3d iTransl_l = leftFootTransform.translation();
  double R00_l = iRotation_l(0,0); double R01_l = iRotation_l(0,1); double R10_l = iRotation_l(1,0);
  double R11_l = iRotation_l(1,1); double R00_r = iRotation_l(0,0); double R01_r = iRotation_l(0,1);
  double R10_r = iRotation_l(1,0); double R11_r = iRotation_l(1,1);
  double tx_l = iTransl_l(0); double tx_r = iTransl_r(0); double ty_l = iTransl_l(1);
  double ty_r = iTransl_r(1);  

  Matrixd<2,56+12> C1 = Matrixd<2,56+12>::Zero();
  Matrixd<2,56+12> C2 = Matrixd<2,56+12>::Zero();

  // Cl
  C1(0,56) = R01_l;C1(0,57) = -R00_l; C1(0,61) = dx; C1(0,62) = R01_r; C1(0,63) = -R00_r; C1(0,67) = dx;
  C1(1,56) = R11_l;C1(1,57) = -R10_l; C1(1,61) = dy; C1(1,62) = R11_r; C1(1,63) = -R10_r; C1(1,67) = dy;

  // Cu
  C2(0,56) = R01_l;C2(0,57) = -R00_l; C2(0,61) = -dx; C2(0,62) = R01_r; C2(0,63) = -R00_r; C2(0,67) = -dx;
  C2(1,56) = R11_l;C2(1,57) = -R10_l; C2(1,61) = -dy; C2(1,62) = R11_r; C2(1,63) = -R10_r; C2(1,67) = -dy;

  Matrixd<2, 1>    l2, l1;
  Matrixd<2, 1>    u2, u1;
  
  u2(0,0) = -tx_l-tx_r;
  u2(1,0) = -ty_l-ty_r;

  l2(0,0) = -high_val;
  l2(1,0) = -high_val;

  u1 << high_val, high_val;
  l1(u2);

  // friction cone constraints
  double mu = 0.5;
  Matrixd<2,56+12> C3 = Matrixd<2,56+12>::Zero();
  // 59: Fx_l 61: Fz_l
  // 65: Fx_r 67: Fz_r
  C3(0,59) = 1.0; C3(0,61) = -mu;
  C3(1,65) = 1.0; C3(1,67) = -mu;

  Matrixd<2,1> u3 = Matrixd<2,1>::Zero();
  Matrixd<2,1> l3 = -high_val*Matrixd<2,1>::Ones();

  // lower priority: keep a certain joint configuration: avoid null space movements
  double Kp_j = 5.0;
  double Kd_j = 1;

  // B4 y + b4 = 0
  Matrixd<50,56+12> B4 = Matrixd<50,56+12>::Zero();
  B4.block(0,6,50,50) = Eigen::Matrix<double,50,50>::Identity();
  Matrixd<50,1>  b4 = -Kp_j*(initialConfiguration.tail(50)-mRobot->getPositions().tail(50))+Kd_j*q_dot.tail(50);
  
  // regularizer on contact forces
  Matrixd<12,56+12> B5 = Matrixd<12,56+12>::Zero();
  B5.block(0,56,12,12) = Matrixd<12,12>::Identity();
  Matrixd<12,1> b5 = Matrixd<12,1>::Zero();

  
  // feasibility
  solver.add_eq_constr(B1, - N_l,0);
  solver.set_solve_type(0, hrc::solve_type::Pinv);    

  // feet
  solver.add_eq_constr(B2, -b2, 1);

  // centroidal 
  solver.add_eq_constr(B3,-b3,2);

  // ZMP
  solver.add_ineq_contstr(C2, l2, u2, 2);
  solver.add_ineq_contstr(C1, l1, u1, 2);

  // friction cone
  //solver.add_ineq_contstr(C3,l3,u3, 2);

  // posture
  solver.add_eq_constr(B4,-b4,3);

  // GRF regularizer
  solver.add_eq_constr(B5,-b5,3);

  Matrixd<56+12, 1> y_mine = solver.solve();

  Matrixd<50, 1> t_des = M_u*y_mine.head(56) + N_u - J_contact_u.transpose()*y_mine.tail(12);

  Eigen::MatrixXd complete_sol = Eigen::MatrixXd::Zero(56+12+50,1);
  complete_sol << t_des, y_mine;
  
  y_file << complete_sol.transpose() << "\n";
  com_file << current.com.pos.transpose()<<"\n";
  com_des_file << desired.com.pos.transpose() << "\n";

  /* ZMP from solution */
  ty_l = y_mine(57,0);
  tx_l = y_mine(56,0);
  ty_r = y_mine(63,0);
  tx_r = y_mine(62,0);
  double Fz_l = y_mine(61,0);
  double Fz_r = y_mine(67,0);

  Eigen::Vector3d cop_l_l; cop_l_l << -ty_l / Fz_l, tx_l/Fz_l, 0.0;
  Eigen::Vector3d cop_r_r; cop_r_r << -ty_r / Fz_r, tx_r/Fz_r, 0.0;
  
  //Eigen::Vector3d cop_l_w = iTransl_l + iRotation_l*cop_l_l;
  //Eigen::Vector3d cop_r_w = iTransl_r + iRotation_r*cop_r_r;
  Eigen::Vector3d cop_l_w = cop_l_l;
  Eigen::Vector3d cop_r_w = cop_r_r;

  Eigen::Vector3d zmp = (Fz_l*cop_l_w + Fz_r*cop_r_w)/(Fz_l + Fz_r) ;

  ZMP_sol_file<<zmp.transpose()<<"\n";
  ZMP_file << getZmpFromExternalForces().transpose()<<"\n";


  return t_des;

  //return y_mine.head(56).tail(50);
    
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

    state.zmpPos = getZmpFromExternalForces(); //

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

State Controller::getDesiredRobotState(int timeStep) {
  State state;
  // just COM for now

  double x_ampl = 0.10; 
  double y_ampl = 0.;   
  double x_freq = 2*M_PI/500; 
  double y_freq = 2*M_PI/500;

  double x_targ = x_ampl*std::sin(x_freq*timeStep);
  double y_targ = y_ampl*std::cos(y_freq*timeStep);

  state.com.pos = initial_com ;//+ (Eigen::Vector3d() << x_targ,y_targ,0.0).finished();

  state.leftFoot.pos = initial.getLeftFootPose().tail(3);
  state.rightFoot.pos = initial.getRightFootPose().tail(3);
  state.leftFoot.ang_pos = initial.getLeftFootPose().head(3);
  state.rightFoot.ang_pos = initial.getRightFootPose().head(3);
  return state;
}

Eigen::Vector3d Controller::getZmpFromExternalForces()
{
  Eigen::Vector3d zmp_v;
  bool left_contact = false;
  bool right_contact = false;

  Eigen::Vector3d left_cop;
  if(abs(mLeftFoot->getConstraintImpulse()[5]) > 0.01){
    left_cop << -mLeftFoot->getConstraintImpulse()(1)/mLeftFoot->getConstraintImpulse()(5), 
                mLeftFoot->getConstraintImpulse()(0)/mLeftFoot->getConstraintImpulse()(5), 
                0.0;
    Eigen::Matrix3d iRotation = mLeftFoot->getWorldTransform().rotation();
    Eigen::Vector3d iTransl   = mLeftFoot->getWorldTransform().translation();
    left_cop = iTransl + iRotation*left_cop;
    left_contact = true;
  }

  Eigen::Vector3d right_cop;
  if(abs(mRightFoot->getConstraintImpulse()[5]) > 0.01){
    right_cop << -mRightFoot->getConstraintImpulse()(1)/mRightFoot->getConstraintImpulse()(5), 
                 mRightFoot->getConstraintImpulse()(0)/mRightFoot->getConstraintImpulse()(5), 
                  0.0;
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
  if (visualize) {
    COM_viz->setTranslation(current.com.pos);
    COM_des_viz->setTranslation(desired.com.pos);
    ZMP_viz->setTranslation(current.zmpPos);
  }
}

void Controller::setInitialConfiguration() {
  initialConfiguration = mRobot->getPositions();

  // floating base
  mRobot->setPosition(0, 0.0 );
  mRobot->setPosition(1, -0*M_PI/180.0);
  mRobot->setPosition(2, 0.0 );
  mRobot->setPosition(3, 0.0);
  mRobot->setPosition(4, 0.0);
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


dart::dynamics::SimpleFramePtr Controller::registerBall(const std::string& name, const Eigen::Vector3d& color) {
  dart::dynamics::SimpleFramePtr ball_frame = std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World(),name, Eigen::Isometry3d::Identity());

  ball_frame->setShape(std::make_shared<dart::dynamics::EllipsoidShape>(.1 * Eigen::Vector3d::Ones()));
  ball_frame->createVisualAspect();
  if (visualize)
    ball_frame->getVisualAspect()->setColor( color );
  mWorld->addSimpleFrame(ball_frame);

  return ball_frame;
}
