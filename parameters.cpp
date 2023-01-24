#pragma once

#include<cmath>

// Times
const double timeStep = 0.01;
const int doubleSupportSamples = 20;

// Walk parameters
const double stepHeight = 0.03;
const double comTargetHeight = 0.75;
const double omega = sqrt(9.81/comTargetHeight);

// Footstep planner
const double ell = 0.2;
const double coronalDeviationMax = 0.05;
const double sagittalDeviationMax = 0.5;
const double alpha = 0.1;
const double averageVel = 0.05;
const double averageTime = 0.5;
const double maxStepDuration = 2.0;
const double minStepDuration = 0.6;
const double thetaMax = 0.30;
const double footConstraintSquareWidth = 0.08;
