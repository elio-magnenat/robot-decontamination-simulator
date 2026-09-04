#ifndef PIN_2026_ALPINE_EXPLOSION_H
#define PIN_2026_ALPINE_EXPLOSION_H
#include "particles.h"
#include "common/Movement.h"
#include "common/State.h"

/**
 * @brief looks for the time of the next explosion inside a time interval
 *
 * @param particles : particles on the board
 * @param after : lower time bound
 * @param limit : upper time bound
 * @param outTime : (output) tells when the explosion will happen
 * @return True if there is an explosion inside the given time interval
 */
inline bool nextExplosionTime(const Particles &particles, const Time after, const Time limit, Time &outTime) {
    bool found = false;
    for (const ParticleInfo &p: particles) {
        //if the particle never explodes -> continue
        if (p.explosionTimes.empty() || p.explosionTimes.front().empty()) continue;

        const Time t = p.explosionTimes.front().front(); // = when the particle explodes
        if (t <= after + EPS || t > limit + EPS) continue; // not inside our dt -> continue
        if (!found || t < outTime) { // if it is the first explosion found, or a closer one
            outTime = t;
            found = true;
        }
    }
    return found;
}

/**
 * @brief makes the particles explode when their time has come
 *
 * @param state : state of the simulation
 * @param now : current time
 * @param nextParticleId : free id for the new particles
 * @return True if at least one particle has split or disappeared
 */
inline bool explodeDueParticles(State &state, const Time now, UniqueId &nextParticleId) {
    bool exploded = false;

    for (auto it = state.particles.begin(); it != state.particles.end();) {
        //particle with a timer and the timer is over -> explosion
        if (!it->explosionTimes.empty() && !it->explosionTimes.front().empty()
            && it->explosionTimes.front().front() <= now + EPS) {
            exploded = true;
            if (it->explosionTimes.size() == 1) {
                it = state.particles.erase(it); //last explosion = the particle is removed
            } else { //split
                const UniqueId freeId = firstFreeParticleId(state.particles);
                if (freeId > nextParticleId) nextParticleId = freeId;
                //creation of the 4 children
                auto kids = splitParticleWithTimes(*it, nextParticleId); // + slice remaining levels into kids
                nextParticleId += 4;
                it = state.particles.erase(it);
                it = state.particles.insert(it, kids.begin(), kids.end());
            }
        } else {
            ++it;
        }
    }
    return exploded;
}

#endif //PIN_2026_ALPINE_EXPLOSION_H
