#ifndef ROBOTS_DECONTAMINATEURS_ALPINE_MOVEMENT_H
#define ROBOTS_DECONTAMINATEURS_ALPINE_MOVEMENT_H

#include "State.h"
#include "Robot.h"
#include <cmath>

#include "Constants.h"
#include "angle_utils.h"

// Where a robot ends up after deltaTime, given its two wheel speeds.
//
// The robot is a differential drive: the wheels are on the two sides, spaced
// by the diameter (2 * radius). Three cases come out of the same equations:
//   - equal speeds        -> straight line, heading does not change
//   - opposite speeds     -> spins on the spot, position does not change
//   - anything else       -> circular arc
//
// Speeds are assumed constant over deltaTime, which is true because commands
// only change at multiples of CommandTimeInterval.
//
// This does not handle collisions, decontamination or explosions: it is pure
// motion. deltaTime may be negative to go back in time.
inline RobotInfo getRobotAfterTime(RobotInfo const& robot, Time deltaTime)
{
    RobotInfo result = robot;
    const double vg = robot.leftSpeed;
    const double vd = robot.rightSpeed;
    const double alpha = robot.angle * M_PI / 180.0;  // JSON stores degrees

    if (std::fabs(vd-vg) < EPS)
    {
        // Straight line. The general formula below would divide by zero here.
        const double v = 0.5 * (vg + vd);
        result.position.x += v * deltaTime * std::cos(alpha);
        result.position.y += v * deltaTime * std::sin(alpha);
        result.angle = normalizeDeg360(robot.angle); // same heading, kept in [0,360[
        return result;
    }

    // Circular arc. omega is the turn rate, R the signed turn radius.
    // Spinning on the spot (vg == -vd) needs no special case: R becomes 0 and
    // the formulas leave the position untouched.
    const double omega = (vd - vg) / (2.0 * robot.radius);
    const double alpha2 = alpha + omega * deltaTime;
    const double R = robot.radius * (vg + vd) / (vg - vd);

    result.position.x += R * (std::sin(alpha) - std::sin(alpha2));
    result.position.y += R * (std::cos(alpha2) - std::cos(alpha));
    result.angle = normalizeDeg360(alpha2 * 180.0 / M_PI);
    return result;
}

// Convenience: move the whole State at once. Only robots move.
inline State getStateAfterTime(State const& state, Time deltaTime)
{
    State result = state;
    for (RobotInfo& robot : result.robots)
        robot = getRobotAfterTime(robot, deltaTime);

    result.time = state.time + deltaTime;
    return result;
}


#endif //ROBOTS_DECONTAMINATEURS_ALPINE_MOVEMENT_H
