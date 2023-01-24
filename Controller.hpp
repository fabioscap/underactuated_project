#pragma once

#include <Eigen/Core>
#include "dart/dart.hpp"
#include  <fstream>
#include "utils.cpp"
#include "types.hpp"
#include "FootstepPlan.hpp"
#include <string>
#include "computeReferenceTrajectory.hpp"
#include <labrob_qpsolvers/qpsolvers.hpp>

class Controller
{
public:
  Controller(dart::dynamics::SkeletonPtr _robot, dart::simulation::WorldPtr _world);
  virtual ~Controller();

  void update();
  
  void setInitialConfiguration();

  Eigen::Vector3d getZmpFromExternalForces();

  

  Eigen::MatrixXd getJacobian();
  Eigen::MatrixXd getJacobianDeriv();

  

  Eigen::VectorXd getJointAccelerations(State desired, State current, WalkState walkState);
  Eigen::VectorXd getJointVelocities(State desired, State current, WalkState walkState);
  Eigen::VectorXd getJointAccelerationsQP2(State desired, State current, WalkState walkState);
  Eigen::VectorXd getJointAccelerationsQPJointLevel(State desired, State current, WalkState walkState);
  Eigen::VectorXd getJointTorques(State desired, State current, WalkState walkState);
  Eigen::VectorXd getJointTorquesSimple(State desired, State current, WalkState walkState);
  State getCurrentRobotState();
  State getDesiredRobotState(int i);

private:
  dart::dynamics::SkeletonPtr mRobot;

  dart::dynamics::BodyNode* mTorso;

  dart::simulation::WorldPtr mWorld;

  dart::dynamics::BodyNode* mLeftFoot;
  dart::dynamics::BodyNode* mRightFoot;
  dart::dynamics::BodyNode* mSupportFoot;
  dart::dynamics::BodyNode* mSwingFoot;
  dart::dynamics::BodyNode* mBase;

  State desired;
  State current;
  State initial;
  WalkState walkState;

  FootstepPlan* footstepPlan;

  std::vector<Logger*> logList;

  std::vector<State> ref;

  Eigen::VectorXd initialConfiguration;

public:

};
