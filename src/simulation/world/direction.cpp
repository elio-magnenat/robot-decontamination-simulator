#include "direction.h"

#include "common/angle_utils.h"
#include "common/Constants.h"
#include "common/Constraint.h"

using namespace std;

pair<RobotInfo, double> moveRobot(const RobotInfo& info, const Position& pos, const Constraint& constraint) {
    const Speed maneuver = min(constraint.maxForwardSpeed, constraint.maxBackwardSpeed);
    RobotInfo newInfo = info;

    const double distance = hypot(info.position.x - pos.x, info.position.y - pos.y);
    const double targetAngleDeg = atan2(pos.y - info.position.y, pos.x - info.position.x) * RAD2DEG;
    const double diff = normalizeDeg(targetAngleDeg - info.angle);

    if (std::abs(diff) > MAX_ARC_TURN_DEG) {
        // target too much on the side -> turn on the spot
        const double remaining = std::max(0.0, std::abs(diff) - info.captureAngle);

        const double neededSpeed = info.radius * (remaining * DEG2RAD) / constraint.CommandTimeInterval;
        const Speed turnSpeed = std::min(maneuver, neededSpeed);

        const bool turnPositive = (std::abs(diff) > 179.0) ? (info.leftSpeed < info.rightSpeed) : (diff > 0);
        //we turn to the left or to the right
        newInfo.leftSpeed = turnPositive ? -turnSpeed : turnSpeed;
        newInfo.rightSpeed = turnPositive ? turnSpeed : -turnSpeed;
    }
    else if (std::abs(diff) > ANGLE_TOLERANCE_DEG) {
        const double remaining = diff -  std::copysign(info.captureAngle, diff);
        //target a bit on the side, we go there with a curve
        const double omega = (remaining * DEG2RAD) / constraint.CommandTimeInterval;
        const Speed outer = constraint.maxForwardSpeed;
        double inner = outer - 2.0 * info.radius * std::abs(omega);

        if (inner < -constraint.maxBackwardSpeed) inner = -constraint.maxBackwardSpeed;

        // diff > 0 : turn left, so the left wheel is slowed down.
        newInfo.leftSpeed = (diff > 0) ? inner : outer;
        newInfo.rightSpeed = (diff > 0) ? outer : inner;
    }
    else {
        //straight line -> the robot already looks at the target
        newInfo.leftSpeed = constraint.maxForwardSpeed;
        newInfo.rightSpeed = constraint.maxForwardSpeed;
    }
    //time we think it takes to cover the distance if all goes well (and straight)
    const double total_time = distance / constraint.maxForwardSpeed;
    return {newInfo, total_time};
}
