#ifndef ROBOTS_POSITION_H
#define ROBOTS_POSITION_H

#include "json_util.h"

struct Position {
   // default to 0
   double x = 0.;
   double y = 0.;

   NLOHMANN_DEFINE_TYPE_INTRUSIVE(Position, x, y)
   JSON_DEFINE_FLUX_OPERATORS(Position)
};

#endif //ROBOTS_POSITION_H
