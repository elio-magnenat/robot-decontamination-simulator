#ifndef PIN_2026_ALPINE_TRAJECTORY_COLLISION_H
#define PIN_2026_ALPINE_TRAJECTORY_COLLISION_H

#include "trajectory_simulator.h"

// Simulates a robot movement for a given command and duration and detect collisions or particle capture
SimulatedCandidate simulateCandidate(const RobotInfo& robot,const WheelCommand& command,const ParticleInfo& particle,const WorldBounds& world,Time duration);

#endif //PIN_2026_ALPINE_TRAJECTORY_COLLISION_H