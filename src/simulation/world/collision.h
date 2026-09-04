#ifndef PIN_2026_ALPINE_COLLISION_H
#define PIN_2026_ALPINE_COLLISION_H

#include "common/Particle.h"
#include "common/Robot.h"
#include "common/State.h"
#include "common/Types.h"

/**
 * @brief the distance between two robots
 *
 * @param a : robot a
 * @param b : robot b
 * @return the distance, negative if the robots overlap, 0 -> they touch
 */
double robotGap(const RobotInfo& a, const RobotInfo& b);

/**
 * @brief distance between a wall and a robot
 *
 * @param robot : the robot we check
 * @param origin : start of the game field
 * @param end : end of the game field
 * @return the distance to the closest wall
 */
double wallGap(const RobotInfo& robot, const Position& origin, const Position& end);

/**
 * @brief Predicts when the first robot will hit a wall
 *
 * @param robots : list of the robots
 * @param origin : start of the game field
 * @param end : end of the game field
 * @param limit : dt of the prediction
 * @param outTime : time of the collision (output)
 * @param outRobot : id of the robot involved (output)
 * @return True if a collision with a wall is found inside this dt
 */
bool nextWallContact(const Robots& robots, const Position& origin, const Position& end, Time limit, Time& outTime,
                     int& outRobot);
/**
 * @brief checks if two robots touch each other
 *
 * @param a : robot a
 * @param b : robot b
 * @return True if the gap between them is zero or negative (so they touch)
 */
bool areTouching(const RobotInfo& a, const RobotInfo& b);

/**
 * @brief finds the FIRST collision between two robots
 *
 * @param robots : list of the robots to test (tested by pairs)
 * @param maxTime : max prediction time
 * @param outTime : when ? (output)
 * @param outA : id of the robot a involved (output)
 * @param outB : id of the robot b involved (output)
 * @return True if a collision is coming
 */
bool findFirstCollision(const Robots& robots, Time maxTime, Time& outTime, int& outA, int& outB);

/**
 * @brief checks if a robot is stuck with another one
 *
 * @param robots : list of the robots
 * @param robot : robot we check the state of
 * @param dt : time on which we watch the moves (getting better ? getting worse ?)
 * @return True if the robot is blocked by a neighbour
 */
bool isBlockedByNeighbour(const Robots& robots, const RobotInfo& robot, Time dt);

/**
 * @brief Checks if a robot is stuck against a wall
 *
 * @param robot : The robot we check
 * @param origin : start of the game field
 * @param end : end of the game field
 * @param dt : lookahead time
 * @return True if the robot is blocked by a border
 */
bool isBlockedByWall(const RobotInfo& robot, const Position& origin, const Position& end, const Time dt);

/**
 * @brief moves every robot and handles the crashes (stops the guilty ones and keeps the others going)
 *
 * @param robots : list of the robots to move
 * @param origin : start of the game field
 * @param end : end of the game field
 * @param dt : interval on which we apply the moves (duration)
 * @return True if at least one collision happened during the move
 */
bool advanceWithCollisions(Robots& robots, Position& origin, Position& end, Time dt);

/**
 * @brief stops the robots right away if they run into each other
 *
 * @param robots : list of the robots
 * @param idA : robot a to stop
 * @param idB : robot b to stop (-1 if there is only one)
 */
void stopRobots(Robots& robots, int idA, int idB);

/**
 * @brief in a colliding pair, stops only the guilty robot(s)
 *
 * A robot turning on the spot does not close the gap : stopping it would freeze
 * it forever while the rules let it turn to get away.
 *
 * @param robots : list of the robots
 * @param idA : first robot of the pair
 * @param idB : second robot of the pair (-1 for a wall collision)
 * @param dt : horizon on which we judge if they get closer
 */
void stopGuiltyRobots(Robots& robots, int idA, int idB, Time dt);

/**
 * @brief predicts when a robot will touch a given particle
 *
 * @param robot : the robot we check
 * @param particle : the particle we check
 * @param maxTime : max prediction time
 * @param outTime : when (output)
 * @return True if a contact is coming inside the given time
 */
bool findParticleContact(const RobotInfo& robot, const ParticleInfo& particle, Time maxTime, Time& outTime);

#endif //PIN_2026_ALPINE_COLLISION_H
