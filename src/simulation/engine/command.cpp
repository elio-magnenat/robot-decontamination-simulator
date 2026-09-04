#include "command.h"

#include <algorithm>
#include <cmath>

#include "world/collision.h"
#include "world/direction.h"
#include "world/particles.h"
#include "common/Constants.h"

/**
 * @brief moves the target away from the robots that are too close
 *
 * @param robots : list of the robots
 * @param robot : robot we compute the target for
 * @param target : the target we want to reach
 * @return the target pushed away from the neighbours
 */
static Position avoidNeighbours(const Robots& robots, const RobotInfo& robot, Position target) {
    double dx = 0.0;
    double dy = 0.0;
    double distance = 0.0;
    double safetyDistance = 0.0;

    for (const RobotInfo& other : robots) {
        if (other.id == robot.id) continue; //skips itself

        dx = robot.position.x - other.position.x;
        dy = robot.position.y - other.position.y;
        distance = std::hypot(dx, dy);
        safetyDistance = (robot.radius + other.radius) * SAFETY_MARGIN;

        //moves the target away if another robot is too close
        if (distance < safetyDistance && distance > 0.0) {
            const double weightRepulsion = (safetyDistance - distance) / safetyDistance;
            target.x += (dx / distance) * weightRepulsion * REPULSION_STRENGTH;
            target.y += (dy / distance) * weightRepulsion * REPULSION_STRENGTH;
        }
    }
    return target;
}

/**
 * @brief applies the speeds to the robot
 *
 * @param robots : list of the robots
 * @param command : robot holding the id and the two speeds to copy
 */
static void setSpeeds(Robots& robots, const RobotInfo& command) {
    for (RobotInfo& robot : robots) {
        if (robot.id == command.id) {
            robot.leftSpeed = command.leftSpeed;
            robot.rightSpeed = command.rightSpeed;
            return;
        }
    }
}

//computes and applies the trajectories for every robot
void applyCommands(Robots& robots, const std::vector<Trajectory>& trajectories, const Constraint& constraint) {
    Position target = {};

    for (const Trajectory& trajectory : trajectories) {
        target = avoidNeighbours(robots, trajectory.robot, trajectory.particle.position);
        setSpeeds(robots, moveRobot(trajectory.robot, target, constraint).first);
    }
}

void unblockRobots(Robots& robots, const Constraint& constraint, const Time dt) {
    const Speed pivot = std::min(constraint.maxForwardSpeed, constraint.maxBackwardSpeed);
    Speed sign = 0.0;
    for (RobotInfo& robot : robots) {
        if (!isBlockedByNeighbour(robots, robot, dt)) continue;

        // FIXME: if two robots had even numbers what happens ?
        //even robots turn one way and odd ones the other way
        sign = (robot.id % 2 == 0) ? 1.0 : -1.0;
        robot.leftSpeed = -sign * pivot;
        robot.rightSpeed = sign * pivot;
    }
}

//Lets the free robots keep going even with collisions around, by stopping the robots that can be blocked between 0 and dt
void cancelBlockedCommands(Robots& robots, const Particles& particles, const Position& origin,
                           const Position& end, const Time dt) {
    Time blockedAt = 0.0;
    Time wallAt = 0.0;
    int blockedA = -1; //id : -1 -> start value pointing to no robot (id :[0;..[)
    int blockedB = -1;
    int wallRobot = -1;
    //we stop the guilty robots (collision between two robots)
    while (findFirstCollision(robots, dt, blockedAt, blockedA, blockedB)
        && blockedAt <= EPS) {
        stopRobots(robots, blockedA, blockedB);
    }

    //we stop the robots on the borders of the field
    while (nextWallContact(robots, origin, end, dt, wallAt, wallRobot)
        && wallAt <= EPS) {
        stopRobots(robots, wallRobot, -1);
    }
    //last check for any other problem inside the given dt
    for (RobotInfo& robot : robots) {
        if (isBlockedByNeighbour(robots, robot, dt)
            || isBlockedByWall(robot, origin, end, dt)
            || isBlockedByParticle(particles, robot, dt)) {
            stopRobots(robots, robot.id, -1);
        }
    }
}
