#include "trajectory_score.h"

#include <algorithm>
#include <cmath>
#include "common/Constants.h"
#include "common/angle_utils.h"

namespace
{
    // Distance covered in one interval. The guard avoids a NaN on empty constraints.
    double reachOf(const EvalContext& ctx)
    {
        const double reach = ctx.constraint.maxForwardSpeed * ctx.constraint.CommandTimeInterval;
        return (reach < EPS) ? 1.0 : reach;
    }

    double centerDistance(const RobotInfo& robot, const ParticleInfo& particle)
    {
        return std::hypot(particle.position.x - robot.position.x,
            particle.position.y - robot.position.y);
    }


    double turnSign(const WheelCommand& command)
    {
        if (std::abs(command.rightSpeed - command.leftSpeed) < EPS) return 0.0;
        return (command.rightSpeed > command.leftSpeed) ? 1.0 : -1.0;
    }

}

// Edge to edge: centre to centre never reaches zero on contact. Clamped at
// zero so overlapping does not score better.
Distance gap(const RobotInfo& robot, const ParticleInfo& particle)
{
    return std::max(0.0, centerDistance(robot, particle) - robot.radius - particle.radius);
}

// Same convention as canDecontaminate: particle minus robot. Swapping them
// shifts the whole world by 180 degrees without looking wrong.
Angle bearingDeg(const RobotInfo& robot, const ParticleInfo& particle)
{
    const double dx = particle.position.x - robot.position.x;
    const double dy = particle.position.y - robot.position.y;
    return std::atan2(dy,dx) * RAD2DEG;
}

// getRobotAfterTime does not normalize its angle, so we do it here. Absolute
// value keeps left and right candidates symmetric.
Angle angleErrorDeg(const RobotInfo& robot, const ParticleInfo& particle)
{
    const double diff = normalizeDeg(bearingDeg(robot, particle) - normalizeDeg(robot.angle));
    return std::abs(diff);
}
double evaluateTrajectoryStep(const RobotInfo& before,
                              const RobotInfo& after,
                              const ParticleInfo& target,
                              const WheelCommand& command,
                              const EvalContext& ctx,
                              ScoreBreakdown* out)
{
    const ScoreWeights& weights = ctx.weights;
    const double reach = reachOf(ctx);

    ScoreBreakdown breakdown;


    // 1. Distance left.
    const double gapBefore = gap(before, target);
    const double gapAfter = gap(after, target);
    breakdown.distance = -weights.distance * gapAfter / reach;

    // 2. Aim. Only what goes past captureAngle counts: inside the cone the robot
    // can already decontaminate, and asking for more would kill the arcs.
    RobotInfo oriented = after;
    oriented.angle = normalizeDeg(after.angle);
    const double excess = std::max(0.0, angleErrorDeg(oriented, target) - oriented.captureAngle);
    breakdown.angle = -weights.angle * excess / SEMI_TURN_DEG;

    // 3. Progress over this step, more telling than absolute distance.
    breakdown.progress = weights.progress * (gapBefore - gapAfter) / reach;

    // 4. Oscillation: turning one way then the other for nothing. Off by default.
    if (ctx.hasPrevious && weights.oscillation > 0.0)
    {
        const double previousSign = turnSign(ctx.previousCommand);
        const double currentSign = turnSign(command);
        if (previousSign != 0.0 && currentSign != 0.0 && previousSign != currentSign)
            breakdown.oscillation = -weights.oscillation;
    }

    breakdown.total = breakdown.distance + breakdown.angle + breakdown.progress + breakdown.oscillation;

    if (out != nullptr) *out = breakdown;
    return breakdown.total;
}

double evaluateTrajectoryStep(const RobotInfo& before,
                              const RobotInfo& after,
                              const ParticleInfo& target,
                              const WheelCommand& command)
{

    // Default constraints: reach is 1, normalization is meaningless.
    const EvalContext ctx{};
    return evaluateTrajectoryStep(before, after, target, command, ctx, nullptr);
}