#pragma once

#include "parameters.cpp"
#include "FootstepPlan.hpp"
#include "types.hpp"
#include "utils.cpp"

std::vector<State> computeReferenceTrajectory(FootstepPlan* plan, State current, int T);
