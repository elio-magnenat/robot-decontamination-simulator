#ifndef ROBOTS_STATE_H
#define ROBOTS_STATE_H

#include <cmath>
#include <vector>

#include "Particle.h"
#include "common/json_util.h"
#include "common/Robot.h"
#include "common/State.h"

struct State {
   Time time = 0.;
   Position worldOrigin;
   Position worldEnd;
   Robots robots;
   Particles particles;

   // Only a finite world with a strictly positive area can be drawn.
   bool hasValidWorld() const {
      return std::isfinite(worldOrigin.x) && std::isfinite(worldOrigin.y)
          && std::isfinite(worldEnd.x)    && std::isfinite(worldEnd.y)
          && worldEnd.x > worldOrigin.x
          && worldEnd.y > worldOrigin.y;
   }

   NLOHMANN_DEFINE_TYPE_INTRUSIVE(State,
                                  time,
                                  worldOrigin,
                                  worldEnd,
                                  robots,
                                  particles)
   JSON_DEFINE_FLUX_OPERATORS(State)
};

using States = std::vector<State>;


#endif //ROBOTS_STATE_H
