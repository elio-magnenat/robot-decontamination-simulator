#ifndef PIN_2026_ALPINE_SIMPLE_TRAJECTORY_H
#define PIN_2026_ALPINE_SIMPLE_TRAJECTORY_H

#include <vector>
#include "trajectory.h"
#include "common/Constraint.h"
#include "common/Robot.h"
#include "common/Particle.h"


//Returns the basic wheel commands for a robot (using the speed limit in the contraint)
std::vector<WheelCommand> getSimpleCommands(const Constraint& constraint);

//Computes a simple predicted trajectory from a robot to a particle (respecting the constraints)
PredictedTrajectory getSimpleTrajectory(const RobotInfo& robot,const ParticleInfo& particle,const Constraint& constraint);

#endif //PIN_2026_ALPINE_SIMPLE_TRAJECTORY_H