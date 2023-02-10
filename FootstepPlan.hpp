#pragma once

#include <iostream>
#include <Eigen/Core>
#include "types.hpp"
#include "parameters.cpp"
#include <vector>
#include "utils.cpp"
#include  <fstream>
#include "parameters.cpp"
#include <labrob_qpsolvers/qpsolvers.hpp>

class FootstepPlan{
    public:
    FootstepPlan();
    ~FootstepPlan();
    void plan(std::vector<Vref> vrefSequence, Eigen::VectorXd initialLeftFoot, Eigen::VectorXd initialRightFoot, bool firstSupportFootIsLeft);
    Eigen::VectorXd getFootstep(int num);
    Eigen::VectorXd getFootstepPosition(int num);
    double getFootstepOrientation(int num);
    int getFootstepStartTiming(int num);
    int getFootstepEndTiming(int num);
    int getFootstepDuration(int num);
    int getFootstepIndexAtTime(int time);
    bool isSingleSupport(int time);
    bool isDoubleSupport(int time);
    bool isAscendingPhase(int time);
    int getSize();
    std::vector<Eigen::VectorXd> getPlan();

    bool isSupportFootLeft(int num);

    private:
    std::vector<Eigen::VectorXd> footstepPlan;
    bool firstSupportFootIsLeft;
};
