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
#include "ISMPC.hpp"
#include <fstream>

class Controller {
public:
  Controller(dart::dynamics::SkeletonPtr _robot, dart::simulation::WorldPtr _world);
  virtual ~Controller();

  void update();
  void draw();
  
  void setInitialConfiguration();

  Eigen::Vector3d getZmpFromExternalForces();

  //Eigen::VectorXd getJointVelocities(State desired, State current, WalkState walkState);
  //Eigen::VectorXd getJointAccelerations(State desired, State current, WalkState walkState);
  Eigen::VectorXd getJointTorques(State& desired,State& current);

  State getCurrentRobotState();
  State getDesiredRobotState(int timeStep);

private:
  // get a pointer to a ball in world (change its coordinates in draw method)
  dart::dynamics::SimpleFramePtr registerBall(const std::string& name, const Eigen::Vector3d& color);

  dart::simulation::WorldPtr mWorld;
  dart::dynamics::SkeletonPtr mRobot;
  
  dart::dynamics::BodyNode* mTorso;
  dart::dynamics::BodyNode* mLeftFoot;
  dart::dynamics::BodyNode* mRightFoot;
  dart::dynamics::BodyNode* mSupportFoot;
  dart::dynamics::BodyNode* mSwingFoot;
  dart::dynamics::BodyNode* mBase;
  
  Eigen::Vector3d comMPC, comMPCdot;
  

  State desired, current, initial;
  Eigen::VectorXd initialConfiguration;
  
  WalkState walkState;

  // FootstepPlan* footstepPlan;

  std::vector<Logger*> logList;

  std::vector<State> ref;
  
  // std::unique_ptr<ISMPC> ismpc;

  Eigen::Vector3d initial_com;
  static constexpr int N_links = 66;
  std::ofstream y_file, com_file, com_des_file, ZMP_sol_file, ZMP_file;

  bool visualize;
  dart::dynamics::SimpleFramePtr COM_viz, COM_des_viz, ZMP_viz;
public:

};
