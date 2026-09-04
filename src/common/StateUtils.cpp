#include "StateUtils.h"

Score getTotalRobotScore(const State& state)
{
    Score totalScore = 0.0;
    for (const RobotInfo& robot : state.robots) {
        totalScore += robot.score;
    }
    return totalScore;
}

Score getRemainingParticleEnergy(const State& state)
{
    Score totalEnergy = 0.0;
    const double pi = std::acos(-1.0);
    for (const ParticleInfo& particle : state.particles) {
        totalEnergy += pi * particle.radius * particle.radius;
    }
    return totalEnergy;
}