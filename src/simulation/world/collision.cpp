#include "collision.h"

#include <cmath>
#include <limits>
#include <stddef.h>

#include "common/Constants.h"
#include "common/Movement.h"

using namespace std;

// Computes the exact free space between two robots
double robotGap(const RobotInfo& a, const RobotInfo& b) {
    return hypot(a.position.x - b.position.x, a.position.y - b.position.y)
        - (a.radius + b.radius);
}

// Computes the exact free space between a robot and the closest wall
double wallGap(const RobotInfo& robot, const Position& origin, const Position& end) {
    // returns the wall that is the closest to the robot
    const double left = robot.position.x - robot.radius - origin.x;
    const double bottom = robot.position.y - robot.radius - origin.y;
    const double right = end.x - robot.radius - robot.position.x;
    const double top = end.y - robot.radius - robot.position.y;

    return min(min(left, bottom), min(right, top));
}

/**
 * @brief Checks if the robot is going away from the wall
 *
 * @param robot : the robot we check
 * @param origin : start of the game field
 * @param end : end of the game field
 * @param dt : lookahead time
 * @return True if the gap never gets smaller
 */
static bool wallDoesNotApproach(const RobotInfo& robot, const Position& origin,
                                const Position& end, const Time dt) {
    double previousGap = wallGap(robot, origin, end);

    for (size_t sample = 1; sample <= GAP_SAMPLES; ++sample) {
        const Time t = (dt * sample) / GAP_SAMPLES;
        const double gap = wallGap(getRobotAfterTime(robot, t), origin, end);
        //if the gap gets smaller -> the robot goes closer to the wall
        if (gap < previousGap - EPS) return false;
        previousGap = gap;
    }
    return true; //it goes away or it does not move
}

// predicts when the next collision with a wall will happen
bool nextWallContact(const Robots& robots, const Position& origin, const Position& end, const Time limit, Time& outTime, int& outRobot) {
    bool found = false;
    outTime = limit;

    for (const RobotInfo& robot : robots) {
        Time hit = -1.0;
        Time lo = 0.0;
        Time hi = 0.0;

        //already touching
        if (wallGap(robot, origin, end) <= CONTACT_EPS) {
            if (wallDoesNotApproach(robot, origin, end, limit)) continue;

            outTime = 0.0;
            outRobot = robot.id;
            return true;
        }

        //finds roughly when the wall is touched
        for (size_t sample = 1; sample <= GAP_SAMPLES; ++sample) {
            const Time t = limit * sample / GAP_SAMPLES;
            if (wallGap(getRobotAfterTime(robot, t), origin, end) <= 0.0) {
                hit = t;
                break;
            }
        }
        if (hit < 0.0) continue;

        //finds exactly when (dichotomy search)
        lo = hit - limit / GAP_SAMPLES;
        hi = hit;
        if (lo < 0.0) lo = 0.0;

        while (hi - lo > numeric_limits<float>::epsilon()) {
            const Time mid = 0.5 * (lo + hi);
            if (wallGap(getRobotAfterTime(robot, mid), origin, end) <= 0.0) {
                hi = mid;
            }
            else {
                lo = mid;
            }
        }

        //earlier collision -> we update
        if (!found || lo < outTime) {
            found = true;
            outTime = lo;
            outRobot = robot.id;
        }
    }
    return found;
}

//tells if two robots touch each other
bool areTouching(const RobotInfo& a, const RobotInfo& b) {
    return robotGap(a, b) <= CONTACT_EPS;
}

//written lower : tells if `mover` closes the gap on its own, `other` being frozen
static bool robotClosesGap(const RobotInfo& mover, const RobotInfo& other, Time dt);

/**
 * @brief tests if two touching robots are going away from each other
 *
 * @param a : robot a
 * @param b : robot b
 * @param dt : lookahead time
 * @return True if the gap never gets smaller
 */
static bool doesNotApproach(const RobotInfo& a, const RobotInfo& b, Time dt) {
    double previousGap = robotGap(a, b);

    for (size_t sample = 1; sample <= GAP_SAMPLES; ++sample) {
        const Time t = (dt * sample) / GAP_SAMPLES;
        const double gap = robotGap(getRobotAfterTime(a, t), getRobotAfterTime(b, t));

        if (gap < previousGap - EPS) return false;
        previousGap = gap;
    }
    return true;
}

bool isBlockedByNeighbour(const Robots& robots, const RobotInfo& robot, const Time dt) {
    for (const auto& other : robots) {
        if (other.id == robot.id) continue;
        if (areTouching(robot, other) && robotClosesGap(robot, other, dt)) return true;
    }
    return false;
}

bool isBlockedByWall(const RobotInfo& robot, const Position& origin, const Position& end, const Time dt) {
    return wallGap(robot, origin, end) <= CONTACT_EPS
        && !wallDoesNotApproach(robot, origin, end, dt);
}

// predicts when a collision between two robots happens
bool findFirstCollision(const Robots& robots, const Time maxTime, Time& outTime, int& outA, int& outB) {
    bool found = false;
    outTime = maxTime;

    for (size_t i = 0; i < robots.size(); ++i) {
        for (size_t j = i + 1; j < robots.size(); ++j) {
            const RobotInfo& a = robots[i];
            const RobotInfo& b = robots[j];

            //already touching
            if (areTouching(a, b)) {
                if (doesNotApproach(a, b, maxTime)) continue;

                found = true;
                outTime = 0.0;
                outA = a.id;
                outB = b.id;
                return true;
            }

            //sampling
            Time hit = -1.0;
            for (size_t sample = 1; sample <= GAP_SAMPLES; ++sample) {
                const Time t = maxTime * sample / GAP_SAMPLES;
                if (robotGap(getRobotAfterTime(a, t), getRobotAfterTime(b, t)) <= 0.0) {
                    hit = t;
                    break;
                }
            }
            if (hit < 0.0) continue;

            //dichotomy (when exactly ?)
            Time lo = maxTime * (hit / maxTime) - maxTime / GAP_SAMPLES;
            Time hi = hit;
            if (lo < 0.0) lo = 0.0;

            while (hi - lo > numeric_limits<float>::epsilon()) {
                const Time mid = 0.5 * (lo + hi);
                if (robotGap(getRobotAfterTime(a, mid), getRobotAfterTime(b, mid)) <= 0.0) {
                    hi = mid;
                }
                else {
                    lo = mid;
                }
            }

            if (!found || lo < outTime) {
                found = true;
                outTime = lo;
                outA = a.id;
                outB = b.id;
            }
        }
    }
    return found;
}

//stops a given robot right away
void stopRobots(Robots& robots, const int idA, const int idB) {
    for (RobotInfo& robot : robots) {
        if (robot.id == idA || robot.id == idB) {
            robot.leftSpeed = 0.0;
            robot.rightSpeed = 0.0;
        }
    }
}

/**
 * @brief tells if `mover` closes the gap on its own, `other` being frozen
 *
 * @param mover : the robot that moves
 * @param other : the other robot, frozen for the test
 * @param dt : lookahead time
 * @return True if the gap gets smaller, so the mover is guilty
 */
static bool robotClosesGap(const RobotInfo& mover, const RobotInfo& other, const Time dt) {
    RobotInfo frozen = other;
    double previousGap = 0.0;
    size_t sample = 0;

    frozen.leftSpeed = 0.0;
    frozen.rightSpeed = 0.0;
    previousGap = robotGap(mover, frozen);

    for (sample = 1; sample <= GAP_SAMPLES; ++sample) {
        const Time t = (dt * sample) / GAP_SAMPLES;
        const double gap = robotGap(getRobotAfterTime(mover, t), frozen);

        if (gap < previousGap - EPS) return true;
        previousGap = gap;
    }
    return false;
}

void stopGuiltyRobots(Robots& robots, const int idA, const int idB, const Time dt) {
    RobotInfo* a = nullptr;
    RobotInfo* b = nullptr;
    bool guiltyA = false;
    bool guiltyB = false;

    for (RobotInfo& robot : robots) {
        if (robot.id == idA) a = &robot;
        else if (robot.id == idB) b = &robot;
    }

    //Collision against a wall or pair not complete : nothing to decide.
    if (a == nullptr || b == nullptr) {
        stopRobots(robots, idA, idB);
        return;
    }

    guiltyA = robotClosesGap(*a, *b, dt);
    guiltyB = robotClosesGap(*b, *a, dt);

    if (!guiltyA && !guiltyB) {
        stopRobots(robots, idA, idB);
        return;
    }
    if (guiltyA) stopRobots(robots, idA, -1);
    if (guiltyB) stopRobots(robots, idB, -1);
}

//moves the robots, handles the collisions and stops on a crash
bool advanceWithCollisions(Robots& robots, Position& origin, Position& end, const Time dt) {
    Time remaining = dt;
    bool collided = false;
    int guard = 0;

    while (remaining > numeric_limits<float>::epsilon() && guard < 32) {
        Time hitTime = remaining;
        Time wallTime = remaining;
        int idA = -1;
        int idB = -1;
        int wallId = -1;
        bool hasHit = false;
        bool hasWall = false;

        guard++;

        //collision coming soon ?
        hasHit = findFirstCollision(robots, remaining, hitTime, idA, idB);
        hasWall = nextWallContact(robots, origin, end, remaining, wallTime, wallId);

        // No obstacle : the step is done in one go.
        if (!hasHit && !hasWall) {
            for (RobotInfo& r : robots) r = getRobotAfterTime(r, remaining);
            return collided;
        }
        //on a crash -> we take the one that comes first
        if (!hasHit || (hasWall && wallTime < hitTime)) {
            hitTime = wallTime;
            idA = wallId;
            idB = -1;
        }

        //keeps the simulation going until the time of the crash
        for (RobotInfo& r : robots) r = getRobotAfterTime(r, hitTime);

        stopGuiltyRobots(robots, idA, idB, max(remaining - hitTime, dt));

        collided = true;
        remaining -= hitTime; //the time we just simulated does not have to be done again. we keep going for the other robots

        if (hitTime <= numeric_limits<float>::epsilon()) break;
    }
    return collided;
}

// predicts when a robot is going to touch a particle
bool findParticleContact(const RobotInfo& robot, const ParticleInfo& particle, const Time maxTime, Time& outTime) {
    Time hit = -1.0;
    Time lo = 0.0;
    Time hi = 0.0;

    // Already touching at the start of the interval : capture right away
    if (particleGap(robot, particle) <= CONTACT_EPS) {
        outTime = 0.0;
        return true;
    }

    // Sampling : a fast robot can go through a small particle between two bounds
    for (size_t sample = 1; sample <= GAP_SAMPLES; ++sample) {
        const Time t = maxTime * sample / GAP_SAMPLES;
        if (particleGap(getRobotAfterTime(robot, t), particle) <= 0.0) {
            hit = t;
            break;
        }
    }
    if (hit < 0.0) return false;

    //dichotomy
    lo = hit - maxTime / GAP_SAMPLES;
    hi = hit;
    if (lo < 0.0) lo = 0.0;

    while (hi - lo > numeric_limits<float>::epsilon()) {
        const Time mid = 0.5 * (lo + hi);
        if (particleGap(getRobotAfterTime(robot, mid), particle) <= 0.0) {
            hi = mid;
        }
        else {
            lo = mid;
        }
    }
    outTime = lo;
    return true;
}
