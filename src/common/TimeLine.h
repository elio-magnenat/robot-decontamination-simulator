#ifndef ROBOTS_TIMELINE_H
#define ROBOTS_TIMELINE_H

#include <optional>
#include "State.h"
#include "json_util.h"
#include "Movement.h"

struct TimeLine {
    States states;

    const State* getStateAt(Time time) const{
        const State* result = nullptr;

        for (const State& state : states) {
            if (state.time <= time) {
                result = &state;
            } else {
                break;
            }
        }
        return result;
    }

    std::optional<State> getDisplayStateAt(Time time) const{
        const State* baseState = getStateAt(time);
        if (baseState == nullptr) {
            return std::nullopt;
        }
        Time deltaTime = time - baseState->time;

        return getStateAfterTime(*baseState, deltaTime);
    }

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(TimeLine, states)
    JSON_DEFINE_FLUX_OPERATORS(TimeLine)
};

#endif // ROBOTS_TIMELINE_H
