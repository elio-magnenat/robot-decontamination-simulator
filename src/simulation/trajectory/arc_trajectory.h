#ifndef PIN_2026_ALPINE_ARC_TRAJECTORY_H
#define PIN_2026_ALPINE_ARC_TRAJECTORY_H

#include <vector>
#include "trajectory.h"
#include "common/Constraint.h"

// Generates wheel commands for curved movements using the maximum speed for one wheel and slower for the other
std::vector<WheelCommand> getArcCommands(const Constraint& constraint);

#endif // PIN_2026_ALPINE_ARC_TRAJECTORY_H