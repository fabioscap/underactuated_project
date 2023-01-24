#pragma once

#include <Eigen/Core>
#include "utils.cpp"
#include <vector>

struct endEffector {
    Eigen::Vector3d pos, vel, acc, ang_pos, ang_vel, ang_acc;
};

// Contains the state of the LIP robot
struct State {
    endEffector com, leftFoot, rightFoot;
    Eigen::Vector3d zmpPos;

    inline Eigen::VectorXd getComPose() {
	Eigen::VectorXd comPose(6);
        comPose << com.ang_pos, com.pos;
        return comPose;
    }

    inline Eigen::VectorXd getSupportFootPose(bool supportFoot) {
	Eigen::VectorXd sfPose(6);
        if (supportFoot == 0) sfPose << leftFoot.ang_pos, leftFoot.pos;
        else sfPose << rightFoot.ang_pos, rightFoot.pos;
        return sfPose;
    }

    inline Eigen::VectorXd getSupportFootOrientation(bool supportFoot) {
	Eigen::VectorXd sfPose(6);
        if (supportFoot == 0) sfPose << leftFoot.ang_pos, Eigen::Vector3d::Zero();
        else sfPose << rightFoot.ang_pos, Eigen::Vector3d::Zero();
        return sfPose;
    }

    inline Eigen::VectorXd getSwingFootPose(bool supportFoot) {
	Eigen::VectorXd sfPose(6);
        if (supportFoot == 1) sfPose << leftFoot.ang_pos, leftFoot.pos;
        else sfPose << rightFoot.ang_pos, rightFoot.pos;
        return sfPose;
    }

    inline Eigen::VectorXd getComVelocity() {
	Eigen::VectorXd comVelocity(6);
        comVelocity << com.ang_vel, com.vel;
        return comVelocity;
    }
    
    inline Eigen::VectorXd getComAcceleration() {
	Eigen::VectorXd comVelocity(6);
        comVelocity << com.ang_acc, com.vel;
        return comVelocity;
    }

    inline Eigen::VectorXd getSwingFootVelocity(bool supportFoot) {
	Eigen::VectorXd sfVelocity(6);
        if (supportFoot == 1) sfVelocity << Eigen::Vector3d::Zero(), leftFoot.vel;
        else sfVelocity << Eigen::Vector3d::Zero(), rightFoot.vel; //FIXME not zero
        return sfVelocity;
    }

    inline Eigen::VectorXd getRelComPose(bool supportFoot) {
	return vvRel(getComPose(), getSupportFootPose(supportFoot));
    }

    inline Eigen::VectorXd getRelSwingFootPose(bool supportFoot) {
	return vvRel(getSwingFootPose(supportFoot), getSupportFootPose(supportFoot));
    }

    inline Eigen::VectorXd getRelComVelocity(bool supportFoot) {
	return vvRel(getComVelocity(), getSupportFootOrientation(supportFoot));
    }

    inline Eigen::VectorXd getRelSwingFootVelocity(bool supportFoot) {
	return vvRel(getSwingFootVelocity(supportFoot), getSupportFootOrientation(supportFoot));
    }

    //------------

    inline Eigen::VectorXd getLeftFootPose() {
	Eigen::VectorXd pose(6);
        pose << leftFoot.ang_pos, leftFoot.pos;
        return pose;
    }

    inline Eigen::VectorXd getRightFootPose() {
	Eigen::VectorXd pose(6);
        pose << rightFoot.ang_pos, rightFoot.pos;
        return pose;
    }
    
    inline Eigen::VectorXd getLeftFootVelocity() {
	Eigen::VectorXd pose(6);
        pose << leftFoot.ang_vel, leftFoot.vel;
        return pose;
    }

    inline Eigen::VectorXd getRightFootVelocity() {
	Eigen::VectorXd pose(6);
        pose << rightFoot.ang_vel, rightFoot.vel;
        return pose;
    }
    
    inline Eigen::VectorXd getLeftFootAcceleration() {
	Eigen::VectorXd pose(6);
        pose << leftFoot.ang_acc, leftFoot.acc;
        return pose;
    }

    inline Eigen::VectorXd getRightFootAcceleration() {
	Eigen::VectorXd pose(6);
        pose << rightFoot.ang_acc, rightFoot.acc;
        return pose;
    }

    inline Eigen::VectorXd getRelLeftFootPose() {
	return vvRel(getLeftFootPose(), getComPose());
    }

    inline Eigen::VectorXd getRelRightFootPose() {
	return vvRel(getRightFootPose(), getComPose());
    }

    inline Eigen::VectorXd getRelLeftFootVelocity() {
	return vvRel(leftFoot.vel, com.ang_pos);
    }

    inline Eigen::VectorXd getRelRightFootVelocity() {
	return vvRel(rightFoot.vel, com.ang_pos);
    }

    inline Eigen::VectorXd getRelLeftFootAcceleration() {
	return vvRel(leftFoot.acc, com.ang_pos);
    }

    inline Eigen::VectorXd getRelRightFootAcceleration() {
	return vvRel(rightFoot.acc, com.ang_pos);
    }

    //------------

    inline void print() {
        std::cout << "com position: " << com.pos;
        std::cout << "left foot position: " << leftFoot.pos;
        std::cout << "right foot position: " << rightFoot.pos;
    }
};

enum class Foot {LEFT, RIGHT};

struct WalkState {
    Foot supportFoot;
    double simulationTime;
    int iter, footstepCounter, indInitial;
};

inline std::string supportFoot2String(const Foot sf) {
  if(sf == Foot::RIGHT) return "RIGHT";
  else return "LEFT";
}

struct Vref {
    Vref(double _x, double _y, double _omega) : x(_x), y(_y), omega(_omega) {}

    double x = 0;
    double y = 0;
    double omega = 0;
};

struct CandidateJacobians {
    std::vector<Eigen::MatrixXd> jacobians;
};

struct IKgains {
    struct EEgains {
        double roll = 1; double pitch = 1; double yaw = 1; double x = 1; double y = 1; double z = 1;
        
        Eigen::VectorXd getVector() {
            Eigen::VectorXd EEgainsVector(6);
            EEgainsVector << roll, pitch, yaw, x, y, z;
            return EEgainsVector;
        }
        
        void set(double value) {roll = value; pitch = value; yaw = value; x = value; y = value; z = value;}
        
        void setPos(double value) {x = value; y = value; z = value;}
        
        void setAng(double value) {roll = value; pitch = value; yaw = value;}
        
        void setRoll(double value) {roll = value;}
        void setPitch(double value) {pitch = value;}
        void setYaw(double value) {yaw = value;}
        
        void setX(double value) {x = value;}
        void setY(double value) {y = value;}
        void setZ(double value) {z = value;}
    };
    
    EEgains leftFoot, rightFoot, com;
    
    Eigen::MatrixXd getMatrix() {
        Eigen::VectorXd IKgainsVector(18);
        IKgainsVector << leftFoot.getVector(), rightFoot.getVector(), com.getVector();
        return IKgainsVector.asDiagonal();
    }
    
    void set(double value) {
        leftFoot.set(value);
        rightFoot.set(value);
        com.set(value);
    }
};
