//
// Created by niels on 25.08.2026.
//

#ifndef PIN_2026_ALPINE_DIRECTION_H
#define PIN_2026_ALPINE_DIRECTION_H
#include "common/Constraint.h"
#include "common/Robot.h"
#include <cmath>
#include <common/Robot.h>
#include <common/Constraint.h>

/**
 * @brief Computes the motor commands so a robot goes to a target
 *
 * @param info : info of the robot
 * @param pos :  x,y coordinates of the target
 * @param constraint : given limits
 * @return std::pair<RobotInfo, double> :
 *         -first (RobotInfo) : robot with the good speeds (leftSpeed, rightSpeed).
 *         -second (double) : time we think it takes to reach the target.
 */
std::pair<RobotInfo, double> moveRobot(const RobotInfo& info, const Position& pos, const Constraint& constraint);

#endif //PIN_2026_ALPINE_DIRECTION_H
