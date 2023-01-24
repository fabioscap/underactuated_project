#include "computeReferenceTrajectory.hpp"

std::vector<State> computeReferenceTrajectory(FootstepPlan* plan, State current, int T)
{
    std::vector<State> ref_vec;
    double groundHeight = 0.025;

    // Compute candidate ZMP trajectory
    
    Eigen::VectorXd xz(T), yz(T), mc(T);
    Eigen::VectorXd previousFootstepPos, currentFootstepPos, nextFootstepPos, movingConstraint;

    for (int i = 0; i < T; i++) {
        int currentIndex = plan->getFootstepIndexAtTime(i);

        currentFootstepPos = plan->getFootstepPosition(currentIndex);
        nextFootstepPos = plan->getFootstepPosition(currentIndex + 1);
        if (currentIndex == 0) currentFootstepPos = (currentFootstepPos + nextFootstepPos) / 2;

        movingConstraint = currentFootstepPos + (nextFootstepPos - currentFootstepPos) * ramp(i, plan->getFootstepEndTiming(currentIndex) - doubleSupportSamples) / doubleSupportSamples;

        if (plan->isSupportFootLeft(currentIndex)) {
             mc(i) = 1 - ramp(i, plan->getFootstepEndTiming(currentIndex) - doubleSupportSamples) / doubleSupportSamples;
        } else {
             mc(i) = ramp(i, plan->getFootstepEndTiming(currentIndex) - doubleSupportSamples) / doubleSupportSamples;
        }
        
        xz(i) = movingConstraint(0);
        yz(i) = movingConstraint(1);
    }

    // Compute bounded xu and yu
    
    Eigen::VectorXd xu(T);
    Eigen::VectorXd yu(T);
    xu(T-1) = xz(T-1);;
    yu(T-1) = yz(T-1);

    for (int i = T - 2; i >= 0; i--) {
        xu(i) = xu(i+1) * exp(-omega*timeStep) + (1 - exp(-omega*timeStep)) * xz(i);
        yu(i) = yu(i+1) * exp(-omega*timeStep) + (1 - exp(-omega*timeStep)) * yz(i);
    }

    // Compute xs and ys
    
    Eigen::VectorXd xs(T);
    Eigen::VectorXd ys(T);

    xs(0) = 2 * current.com.pos(0) - xu(0);
    ys(0) = 2 * current.com.pos(1) - yu(0);

    for (int i = 1; i < T; i++) {
        xs(i) = xs(i-1) + timeStep * omega * (xz(i-1) - xs(i-1));
        ys(i) = ys(i-1) + timeStep * omega * (yz(i-1) - ys(i-1));
    }

    // Compute foot trajectories

    for (int i = 0; i < T; i++) {
        State state;
    
        int currentIndex = plan->getFootstepIndexAtTime(i);

        if (currentIndex == 0) previousFootstepPos = plan->getFootstepPosition(currentIndex + 1);
        else previousFootstepPos = plan->getFootstepPosition(currentIndex - 1);

        currentFootstepPos = plan->getFootstepPosition(currentIndex);
        nextFootstepPos = plan->getFootstepPosition(currentIndex + 1);
        currentFootstepPos(2) = groundHeight;
        nextFootstepPos(2) = groundHeight;
        previousFootstepPos(2) = groundHeight;

        double actualStepHeight = currentIndex == 0 ? 0 : stepHeight;

        if (i < plan->getFootstepEndTiming(currentIndex) - doubleSupportSamples) { // we are in single support

            double stepCompletion = double(i - plan->getFootstepStartTiming(currentIndex)) /
                   double(plan->getFootstepEndTiming(currentIndex) - plan->getFootstepStartTiming(currentIndex) - doubleSupportSamples);

            double delta = 0.01;
            double dsDuration = double(plan->getFootstepEndTiming(currentIndex) - plan->getFootstepStartTiming(currentIndex) - doubleSupportSamples) * delta;

            if (plan->isSupportFootLeft(currentIndex)) {
                // move right foot
                state.leftFoot.pos = currentFootstepPos;
                state.rightFoot.pos = previousFootstepPos + (nextFootstepPos - previousFootstepPos) * cubic(stepCompletion);
                
                state.leftFoot.vel = Eigen::Vector3d::Zero();
                state.rightFoot.vel = (nextFootstepPos - previousFootstepPos) * cubic_dot(stepCompletion) / dsDuration;
                
                state.leftFoot.acc = Eigen::Vector3d::Zero();
                state.rightFoot.acc = (nextFootstepPos - previousFootstepPos) * cubic_ddot(stepCompletion) / dsDuration / dsDuration;
                
                if (stepCompletion <= 0.5) {
		    state.rightFoot.pos(2) += cubic(stepCompletion*2) * actualStepHeight;
		    state.rightFoot.vel(2) += cubic_dot(stepCompletion*2) * actualStepHeight * 2 / dsDuration;
		    state.rightFoot.acc(2) += cubic_ddot(stepCompletion*2) * actualStepHeight * 4 / dsDuration / dsDuration;
		} else {
		    state.rightFoot.pos(2) += actualStepHeight - cubic((stepCompletion-0.5)*2) * actualStepHeight;
		    state.rightFoot.vel(2) += - cubic_dot((stepCompletion-0.5)*2) * actualStepHeight * 2 / dsDuration;
		    state.rightFoot.acc(2) += - cubic_ddot((stepCompletion-0.5)*2) * actualStepHeight * 4 / dsDuration / dsDuration;
		}
            } else {
                // move left foot
                state.rightFoot.pos = currentFootstepPos;
                state.leftFoot.pos = previousFootstepPos + (nextFootstepPos - previousFootstepPos) * cubic(stepCompletion);
                
                state.rightFoot.vel = Eigen::Vector3d::Zero();
                state.leftFoot.vel= (nextFootstepPos - previousFootstepPos) * cubic_dot(stepCompletion) / dsDuration;
                
                state.rightFoot.acc = Eigen::Vector3d::Zero();
                state.leftFoot.acc= (nextFootstepPos - previousFootstepPos) * cubic_ddot(stepCompletion) / dsDuration / dsDuration;
                
                if (stepCompletion <= 0.5) {
		    state.leftFoot.pos(2) += cubic(stepCompletion*2) * actualStepHeight;
		    state.leftFoot.vel(2) += cubic_dot(stepCompletion*2) * actualStepHeight * 2 / dsDuration;
		    state.leftFoot.acc(2) += cubic_ddot(stepCompletion*2) * actualStepHeight * 4 / dsDuration / dsDuration;
		} else {
		    state.leftFoot.pos(2) += actualStepHeight - cubic((stepCompletion-0.5)*2) * actualStepHeight;
		    state.leftFoot.vel(2) += - cubic_dot((stepCompletion-0.5)*2) * actualStepHeight * 2 / dsDuration;
		    state.leftFoot.acc(2) += - cubic_ddot((stepCompletion-0.5)*2) * actualStepHeight * 4 / dsDuration / dsDuration;
		}
            }
        } else { // we are in double support
            if (plan->isSupportFootLeft(currentIndex)) { 
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
        
        // Orientation (temporarily zero, orientation is fixed)
        
        Eigen::Vector3d xu_ = Eigen::Vector3d(xu(i), yu(i), groundHeight + comTargetHeight);
        Eigen::Vector3d xs_ = Eigen::Vector3d(xs(i), ys(i), groundHeight + comTargetHeight);
        
        state.zmpPos = Eigen::Vector3d(xz(i), yz(i), groundHeight);
        
        state.com.pos = (xu_ + xs_)/2;
        state.com.vel = omega * (xu_ - xs_)/2;
        state.com.acc = omega * omega * (state.com.pos - state.zmpPos - Eigen::Vector3d(0,0,comTargetHeight));
        state.com.ang_pos = Eigen::Vector3d::Zero();
        state.com.ang_vel = Eigen::Vector3d::Zero();
        state.com.ang_acc = Eigen::Vector3d::Zero();

        state.leftFoot.ang_pos = Eigen::Vector3d::Zero();
        state.leftFoot.ang_vel = Eigen::Vector3d::Zero();
        state.leftFoot.ang_acc = Eigen::Vector3d::Zero();

        state.rightFoot.ang_pos = Eigen::Vector3d::Zero();
        state.rightFoot.ang_vel = Eigen::Vector3d::Zero();
        state.rightFoot.ang_acc = Eigen::Vector3d::Zero();
        
        ref_vec.push_back(state);
    }
    
    return ref_vec;
}
