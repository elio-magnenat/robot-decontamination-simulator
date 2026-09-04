#ifndef PIN_2026_ALPINE_TRAJECTORY_PLANNER_H
#define PIN_2026_ALPINE_TRAJECTORY_PLANNER_H

#include "trajectory_simulator.h"

//Estimate the remaining time for a robot to reach and face a particle
double estimateRemainingTime(const RobotInfo& robot,const ParticleInfo& particle,const Constraint& constraint);

//Select the best wheel command for the current robot by evaluating the available movements and their predicted consequences
SimulatedCandidate chooseBestCommand(const RobotInfo& currentRobot,const ParticleInfo& particle,const Constraint& constraint,const WorldBounds& world,Time stepDuration);

#endif //PIN_2026_ALPINE_TRAJECTORY_PLANNER_H