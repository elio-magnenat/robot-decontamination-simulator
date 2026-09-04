//
// Created by niels on 25.08.2026.
//

#ifndef PIN_2026_ALPINE_CONSTRAINT_H
#define PIN_2026_ALPINE_CONSTRAINT_H
#include "json_util.h"
#include <cmath>

struct Constraint {
    double CommandTimeInterval= 0;
    double maxBackwardSpeed = 0;
    double maxForwardSpeed = 0;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(Constraint, CommandTimeInterval, maxForwardSpeed, maxBackwardSpeed)
    JSON_DEFINE_FLUX_OPERATORS(Constraint);
 };

#endif //PIN_2026_ALPINE_CONSTRAINT_H
