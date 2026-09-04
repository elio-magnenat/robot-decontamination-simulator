#ifndef ROBOTS_STATEUTILS_H
#define ROBOTS_STATEUTILS_H
#include <cmath>

#include "common/State.h"
#include "common/Particle.h"

Score getTotalRobotScore(const State& state);

Score getRemainingParticleEnergy(const State& state);

#endif // ROBOTS_STATEUTILS_H