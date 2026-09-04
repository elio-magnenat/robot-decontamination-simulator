//
// Created by niels on 25.08.2026.
//

#ifndef PIN_2026_ALPINE_PARTICLES_H
#define PIN_2026_ALPINE_PARTICLES_H

#include <vector>

#include "collision.h"
#include "common/Constraint.h"
#include "common/Movement.h"
#include "common/Particle.h"
/**
 * @brief finds the smallest free id for a particle
 *
 * @param particles : list of the particles
 * @return a free ID
 */
UniqueId firstFreeParticleId(const Particles& particles);

/**
 * @brief splits a parent particle into 4 children and gives them their next explosions
 *
 * @param parent : particle that explodes
 * @param startingId : first ID used to name the children
 * @return a vector with the 4 children and their info
 */
std::vector<ParticleInfo> splitParticleWithTimes(const ParticleInfo& parent, UniqueId startingId);

/**
 * @brief predicts when the next decontamination will happen
 *
 * @param robots : list of the robots
 * @param particles : list of the particles
 * @param limit : max search time
 * @param outTime : (output) time of the contact
 * @return True if a decontamination happens inside the dt
 */
bool nextDecontaminationTime(const Robots& robots, const Particles& particles, Time limit, Time& outTime);

/**
 * @brief validates the decontamination of the touched particles, removes them and updates the score
 *
 * @param state : global state of the simulation (robots and particles)
 * @return True if at least one particle has been decontaminated
 */
bool decontaminateAtContact(State& state);

/**
 * @brief predicts if a robot touches a particle that it CANNOT decontaminate
 *
 * @param robots : list of the robots
 * @param particles : list of the particles
 * @param limit : max search time
 * @param outTime : (output) time of the contact against the particle
 * @param outRobot : (output) ID of the robot that is going to be blocked
 * @return True if a robot touches a particle it cannot clean
 */
bool nextParticleBlock(const Robots& robots, const Particles& particles, Time limit,Time& outTime, int& outRobot);

/**
 * @brief stops the robots that are stuck against a blocking particle
 *
 * @param state : state of the simulation
 * @return True if at least one robot has been stopped
 */
bool stopRobotsOnParticles(State& state);

/**
 * @brief checks if a robot is stuck against a particle it cannot clean
 *
 * Same idea as isBlockedByWall / isBlockedByNeighbour, but for the particles.
 * Turning on the spot is NOT a block : the rules let a stuck robot turn until
 * it is well oriented, so we only look at the commands that really push the
 * robot into the particle.
 *
 * @param particles : list of the particles
 * @param robot : robot we check the state of
 * @param dt : lookahead time
 * @return True if the robot is blocked by a particle
 */
bool isBlockedByParticle(const Particles& particles, const RobotInfo& robot, Time dt);

/**
 * @brief gives a new trajectory to the stuck robots
 *
 * @param state : state of the simulation
 * @param constraint : constraints given for the simulation
 */
void aimBlockedRobots(State& state, const Constraint& constraint);

#endif //PIN_2026_ALPINE_PARTICLES_H