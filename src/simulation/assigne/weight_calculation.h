//
// Created by Kilian Stockinger on 28.08.2026.
//

#ifndef PIN_2026_ALPINE_WEIGHT_CALCULATION_H
#define PIN_2026_ALPINE_WEIGHT_CALCULATION_H
#include <map>
#include <vector>

#include "../trajectory/trajectory_planner.h"
#include "../trajectory/trajectory_simulator.h"
#include "../trajectory/trajectory.h"
#include "common/Robot.h"
#include "common/Particle.h"

#include "common/State.h"
#include "common/Constraint.h"

#include "../trajectory/trajectory.h"

struct TemporaryWeight {
    //  Weight of the value - Value
    Time timeExpTra;
    Time timeTra;
    Distance sizeParticule;
    int assignedRobots;
};


struct RobotToParticle {
    RobotInfo robotInfo;
    ParticleInfo particleInfo;
    Time togo;
    double weight;
};



std::vector<Trajectory> assignRobotParticule(const State& currentState,
                                             const Constraint& constraint,
                                             std::map<UniqueId, UniqueId>& previousAssignment);

// Met a l'arret les robots qui n'ont recu aucune particule. Sans cible ils
// garderaient sinon les vitesses de l'iteration precedente et continueraient
// a rouler pour rien.
void stopUnassignedRobots(Robots& robots, const std::vector<Trajectory>& assignments);
#endif //PIN_2026_ALPINE_WEIGHT_CALCULATION_H