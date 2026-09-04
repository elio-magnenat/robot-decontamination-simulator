#ifndef PIN_2026_ALPINE_EVENT_H
#define PIN_2026_ALPINE_EVENT_H
#include "common/State.h"

/**
 * @brief tells if we need to make a new state, looking at what happened
 *
 * @param previous : previous state
 * @param current : current state to compare with
 * @return True if a new state must be created
 */
bool isEvent(const State& previous, const State& current) {
    size_t i = 0;

    //a particle is gone = new state
    if (previous.particles.size() != current.particles.size()) return true;

    //some robots are gone (should never happen)
    if (previous.robots.size() != current.robots.size()) return true;

    //did a particle split ? (the IDs do not match anymore)
    for (i = 0; i < current.particles.size(); ++i) {
        if (previous.particles[i].id != current.particles[i].id) return true;
    }

    //we check if the robots changed their move or their score
    for (i = 0; i < current.robots.size(); ++i) {
        const RobotInfo& a = previous.robots[i];
        const RobotInfo& b = current.robots[i];

        //change on the left wheel
        if (std::fabs(a.leftSpeed - b.leftSpeed) > EPS) return true;

        //change on the right wheel
        if (std::fabs(a.rightSpeed - b.rightSpeed) > EPS) return true;

        //decontamination -> the score went up
        if (std::fabs(a.score - b.score) > EPS) return true;
    }
    return false;
}

#endif //PIN_2026_ALPINE_EVENT_H
