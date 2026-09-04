#ifndef PIN_2026_ALPINE_TRAJECTORY_COMMAND_H
#define PIN_2026_ALPINE_TRAJECTORY_COMMAND_H

#include "trajectory.h"
#include "common/Constraint.h"
#include "common/State.h"

/**
 * @brief asks the planner for the best command of each robot and writes it in the state
 *
 * @param state : state of the simulation, the speeds of the robots are changed
 * @param trajectories : pairs (Robot -> target Particle) given by the choice algo
 * @param constraint : given limits
 */
void applyPlannedCommands(State& state, const std::vector<Trajectory>& trajectories,
                          const Constraint& constraint);

/**
 * @brief gives each robot the particle it can reach the fastest
 *
 * A particle already taken by a robot cannot be chosen again.
 *
 * @param state : state of the simulation (robots and particles)
 * @param constraint : given limits
 * @return the pairs (Robot -> target Particle), one for each robot with a target
 */
std::vector<Trajectory> findBestParticlesByEta(const State& state, const Constraint& constraint);

#endif //PIN_2026_ALPINE_TRAJECTORY_COMMAND_H
