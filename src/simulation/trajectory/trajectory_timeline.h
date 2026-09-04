#ifndef PIN_2026_ALPINE_TRAJECTORY_TIMELINE_H
#define PIN_2026_ALPINE_TRAJECTORY_TIMELINE_H

#include "trajectory.h"
#include "common/State.h"
#include "common/TimeLine.h"

//Converts a predicted robot trajectory into a timeline of simulation states
TimeLine trajectoryToTimeline(const State& initialState,const PredictedTrajectory& trajectory);

#endif // PIN_2026_ALPINE_TRAJECTORY_TIMELINE_H