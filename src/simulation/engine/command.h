#ifndef PIN_2026_ALPINE_COMMAND_H
#define PIN_2026_ALPINE_COMMAND_H

#include "../trajectory/trajectory.h"
#include "common/Constraint.h"
#include "common/Particle.h"
#include "common/Robot.h"
#include "common/Types.h"

/**
 * @brief applies the speeds to the robots
 *
 * @param robots :  list of the robots
 * @param trajectories : pairs (Robot -> target Particle) given by the choice algo
 * @param constraint : given limits
 */
void applyCommands(Robots& robots, const std::vector<Trajectory>& trajectories, const Constraint& constraint);

/**
 * @brief moves the stuck robots
 *
 * @param robots : list of the robots that may be stuck
 * @param constraint : given limits
 * @param dt : time step of the simulation
 */
void unblockRobots(Robots& robots, const Constraint& constraint, Time dt);

/**
 * @brief stops the robots right away to avoid collisions
 *
 * @param robots : list of the robots to stop
 * @param particles : list of the particles that can block a robot
 * @param origin : x,y coordinates of the field (start)
 * @param end : x,y coordinates of the field (end)
 * @param dt : lookahead time, we look for a block between 0 and dt
 */
void cancelBlockedCommands(Robots& robots, const Particles& particles, const Position& origin, const Position& end,
                           const Time dt);

#endif //PIN_2026_ALPINE_COMMAND_H
