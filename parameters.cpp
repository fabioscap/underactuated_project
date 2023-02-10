#pragma once

#include<cmath>

// Times
//const double timeStep = 0.01;
//const int doubleSupportSamples = 20;

// Walk parameters
const double stepHeight = 0.08;
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
//const double thetaMax = 0.30;
//const double footConstraintSquareWidth = 0.08;

const double groundHeight = 0.025;

/// ISMPC


// Times
const double worldTimeStep = 0.01;
const double timeStep = 0.01;
const double singleSupportDuration = 0.6;
const double doubleSupportDuration = 0.4;
const double predictionTime = 1.0;
const int prev = 20;


//const double comTargetHeight = 0.78;
// const double kSwingFoot = 0.05;

// Fixed step parameters
//const double stepx = 0.06;
//const double stepy = 0.1;

// Constraints
const double thetaMax = 0.30;
const double footConstraintSquareWidth = 0.08;

// Used in the code
// ****************

const int N = (int)round(predictionTime/timeStep);
const int S = (int)round(singleSupportDuration/timeStep);
const int D = (int)round(doubleSupportDuration/timeStep);
const int M = 4; //ceil(N/(S+D));
const int timesRatio = (int)(timeStep/worldTimeStep); //ceil(N/(S+D));
const int singleSupportSamples = (int)round(singleSupportDuration/worldTimeStep);
const int doubleSupportSamples = (int)round(doubleSupportDuration/worldTimeStep);
//const int doubleSupportSamples = 20;


// IS-MPC parameters
// *****************
const double deltaXMax = 0.25;
const double deltaYIn = 0.15;
const double deltaYOut = 0.28;
// Cost function weights
const double qZd = 1;
const double qVx = 0;
const double qVy = 0;
const double qZ = 100;
const double qF = 10000;//10000000000;


const double robot_mass = 40.0549;
