#ifndef PIN_2026_ALPINE_INTERVAL_H
#define PIN_2026_ALPINE_INTERVAL_H

#include "common/State.h"
#include "common/TimeLine.h"
#include "common/Types.h"
/**
 * @brief moves the simulation step by step over a given dt -> stops if an event happens
 *
 * @param state : current state of the simulation
 * @param timeline : timeline where a new state is added on each event
 * @param from : start time of the dt
 * @param to : end time of the dt
 * @param nextParticleId : id used for the new particles
 * @return True if a collision happened during this dt
 */
bool runInterval(State &state, TimeLine &timeline, Time from, Time to, UniqueId &nextParticleId);

#endif //PIN_2026_ALPINE_INTERVAL_H
