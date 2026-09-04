#ifndef PIN_2026_ALPINE_TRAJECTORY_SCORE_H
#define PIN_2026_ALPINE_TRAJECTORY_SCORE_H

#include "common/Robot.h"
#include "common/Particle.h"
#include "common/Constraint.h"
#include "common/Constants.h"
#include "common/Types.h"
#include "trajectory.h"

// Scoring a single trajectory step. Higher is better: the optimizer keeps the
// best-scoring candidate command.
//
// Only the geometry is judged here (distance left, aim, progress). Captures
// and collisions come from trajectory_simulator; trajectory_planner adds the
// capture bonus and the blocked penalty.
//
// We cannot add pixels to degrees, so lengths are divided by
//     reach = maxForwardSpeed * CommandTimeInterval
// the distance covered in one command interval. The terms then have no unit.
//
// Note on the weights: on a single step distance and progress always move
// together, only their sum (2.5) matters. With that sum, the specification
// example needs w_angle > 1.5, so 2.0 leaves little margin.

struct ScoreWeights {
    // Scale reference: the other weights are read relative to this one.
    double distance = SCORE_WEIGHT_DISTANCE;

    // Do not go below 1.5, see the note above.
    double angle = SCORE_WEIGHT_ANGLE;

    // Relative progress. Only differs from distance when looking ahead.
    double progress = SCORE_WEIGHT_PROGRESS;

    // Far above the other terms, so a real capture always wins.
    double capture = SCORE_WEIGHT_CAPTURE;

    // Touching without capturing: the robot stops and loses one interval.
    double blocked = SCORE_WEIGHT_BLOCKED;

    // Useless change of turn direction. Off while the weight is zero.
    double oscillation = SCORE_WEIGHT_OSCILLATION;
};

struct EvalContext {
    Constraint constraint;
    WheelCommand previousCommand{0, 0}; // used by the oscillation penalty
    bool hasPrevious = false;
    ScoreWeights weights{};
};

struct ScoreBreakdown {
    double distance = 0.0;
    double angle = 0.0;
    double progress = 0.0;
    double oscillation = 0.0;
    double total = 0.0;
};

// Edge to edge distance, never negative.
Distance gap(const RobotInfo& robot, const ParticleInfo& particle);
// Direction from the robot to the particle, in degrees.
Angle bearingDeg(const RobotInfo& robot, const ParticleInfo& particle);
// How far off the robot is aimed, in [0, 180].

Angle angleErrorDeg(const RobotInfo& robot, const ParticleInfo& particle);
double evaluateTrajectoryStep(const RobotInfo& before,
                              const RobotInfo& after,
                              const ParticleInfo& target,
                              const WheelCommand& command,
                              const EvalContext& ctx,
                              ScoreBreakdown* out = nullptr);

// Signature from the specification. reach is 1 here, so debugging only.
double evaluateTrajectoryStep(const RobotInfo& before,
                              const RobotInfo& after,
                              const ParticleInfo& target,
                              const WheelCommand& command);
#endif //PIN_2026_ALPINE_TRAJECTORY_SCORE_H
