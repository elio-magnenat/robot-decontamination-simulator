#include "simple_trajectory.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "common/Constants.h"
#include "common/Movement.h"
#include "common/angle_utils.h"

namespace {
    WheelCommand makeForwardCommand(const Constraint &constraint) {
        return {constraint.maxForwardSpeed, constraint.maxForwardSpeed};
    }

    WheelCommand makeBackwardCommand(const Constraint &constraint) {
        return {-constraint.maxBackwardSpeed, -constraint.maxBackwardSpeed};
    }

    WheelCommand makeLeftRotationCommand(const Constraint &constraint) {
        const Speed rotationSpeed =
                std::min(constraint.maxForwardSpeed, constraint.maxBackwardSpeed);

        return {-rotationSpeed, rotationSpeed};
    }

    WheelCommand makeRightRotationCommand(const Constraint &constraint) {
        const Speed rotationSpeed =
                std::min(constraint.maxForwardSpeed, constraint.maxBackwardSpeed);

        return {rotationSpeed, -rotationSpeed};
    }

    WheelCommand makeStopCommand() {
        return {0.0, 0.0};
    }

    RobotInfo simulateCommand(
        const RobotInfo &robot,
        const WheelCommand &command,
        const Time duration) {
        RobotInfo simulated = robot;

        simulated.leftSpeed = command.leftSpeed;
        simulated.rightSpeed = command.rightSpeed;

        return getRobotAfterTime(simulated, duration);
    }

    void appendStep(
        PredictedTrajectory &trajectory,
        RobotInfo &currentRobot,
        Time &currentTime,
        const WheelCommand &command,
        const Time duration) {
        if (duration <= EPS) {
            return;
        }

        const RobotInfo startRobot = currentRobot;
        const RobotInfo endRobot = simulateCommand(startRobot, command, duration);

        trajectory.steps.push_back({
            currentTime,
            duration,
            command,
            startRobot,
            endRobot
        });

        currentRobot = endRobot;
        currentTime += duration;
    }

    double angleToParticleDeg(
        const RobotInfo &robot,
        const ParticleInfo &particle) {
        const double dx =
                particle.position.x - robot.position.x;

        const double dy =
                particle.position.y - robot.position.y;

        return std::atan2(dy, dx) * RAD2DEG;
    }

    double centerDistance(
        const RobotInfo &robot,
        const ParticleInfo &particle) {
        return std::hypot(
            particle.position.x - robot.position.x,
            particle.position.y - robot.position.y
        );
    }

    double distanceToContact(
        const RobotInfo &robot,
        const ParticleInfo &particle) {
        return std::max(
            0.0,
            centerDistance(robot, particle)
            - robot.radius
            - particle.radius
        );
    }

    bool appendRotationToAngle(
        PredictedTrajectory &trajectory,
        RobotInfo &currentRobot,
        Time &currentTime,
        const double desiredAngleDeg,
        const Constraint &constraint) {
        const Time dt = constraint.CommandTimeInterval;

        const Speed rotationSpeed =
                std::min(
                    constraint.maxForwardSpeed,
                    constraint.maxBackwardSpeed
                );

        if (dt <= EPS ||
            rotationSpeed <= EPS ||
            currentRobot.radius <= EPS) {
            return std::abs(
                       normalizeDeg(
                           desiredAngleDeg - currentRobot.angle
                       )
                   ) <= ANGLE_EPS;
        }

        for (int i = 0; i < MAX_ROTATION_STEPS; ++i) {
            const double errorDeg =
                    normalizeDeg(
                        desiredAngleDeg - currentRobot.angle
                    );

            if (std::abs(errorDeg) <= ANGLE_EPS) {
                return true;
            }

            const WheelCommand fullCommand =
                    errorDeg > 0.0
                        ? makeLeftRotationCommand(constraint)
                        : makeRightRotationCommand(constraint);

            const RobotInfo afterFullTick =
                    simulateCommand(
                        currentRobot,
                        fullCommand,
                        dt
                    );

            // Normalize the angular delta across the -180/180 boundary
            const double fullTurnDeg = normalizeDeg(afterFullTick.angle - currentRobot.angle);

            const double fullTurnMagnitude = std::abs(fullTurnDeg);

            if (fullTurnMagnitude <= ANGLE_EPS) {
                return false;
            }

            if (std::abs(errorDeg) >= fullTurnMagnitude - ANGLE_EPS) {
                appendStep(
                    trajectory,
                    currentRobot,
                    currentTime,
                    fullCommand,
                    dt
                );
            } else {
                // Reduce wheel speeds to avoid overshooting the target angle
                const double ratio =
                        std::abs(errorDeg)
                        / fullTurnMagnitude;

                const WheelCommand adjustedCommand{
                    fullCommand.leftSpeed * ratio,
                    fullCommand.rightSpeed * ratio
                };

                appendStep(
                    trajectory,
                    currentRobot,
                    currentTime,
                    adjustedCommand,
                    dt
                );
            }
        }

        return false;
    }

    void waitUntilNextCommandTick(
        PredictedTrajectory &trajectory,
        RobotInfo &currentRobot,
        Time &currentTime,
        const Constraint &constraint) {
        const Time dt =
                constraint.CommandTimeInterval;

        if (dt <= EPS) {
            return;
        }

        const double tickIndex =
                std::ceil(
                    (currentTime - EPS) / dt
                );

        const Time nextTick =
                tickIndex * dt;

        const Time wait =
                nextTick - currentTime;

        // Wait until the next valid command tick before rotating
        if (wait > EPS) {
            appendStep(
                trajectory,
                currentRobot,
                currentTime,
                makeStopCommand(),
                wait
            );
        }
    }

    PredictedTrajectory makeInvalidTrajectory(
        const RobotInfo &robot,
        const ParticleInfo &particle) {
        PredictedTrajectory result;

        result.robotId = robot.id;
        result.particleId = particle.id;

        return result;
    }

    PredictedTrajectory buildForwardBaseline(
        const RobotInfo &robot,
        const ParticleInfo &particle,
        const Constraint &constraint) {
        PredictedTrajectory result =
                makeInvalidTrajectory(robot, particle);

        RobotInfo currentRobot = robot;
        Time currentTime = 0.0;

        // Face the particle then drive straight to contact
        if (canDecontaminate(
            currentRobot,
            particle)) {
            result.captured = true;
            result.eta = 0.0;
            result.captureTime = 0.0;

            return result;
        }

        if (constraint.maxForwardSpeed <= EPS) {
            return result;
        }

        const double targetAngle =
                angleToParticleDeg(
                    currentRobot,
                    particle
                );

        if (!appendRotationToAngle(
            result,
            currentRobot,
            currentTime,
            targetAngle,
            constraint)) {
            return makeInvalidTrajectory(
                robot,
                particle
            );
        }

        const double remainingDistance =
                distanceToContact(
                    currentRobot,
                    particle
                );

        if (remainingDistance > EPS) {
            const Time travelTime =
                    remainingDistance
                    / constraint.maxForwardSpeed;

            appendStep(
                result,
                currentRobot,
                currentTime,
                makeForwardCommand(constraint),
                travelTime
            );
        }

        if (canDecontaminate(
            currentRobot,
            particle)) {
            result.captured = true;

            result.eta =
                    currentTime;

            result.captureTime =
                    currentTime;
        }

        return result;
    }

    PredictedTrajectory buildBackwardBaseline(
        const RobotInfo &robot,
        const ParticleInfo &particle,
        const Constraint &constraint) {
        PredictedTrajectory result =
                makeInvalidTrajectory(
                    robot,
                    particle
                );

        RobotInfo currentRobot = robot;
        Time currentTime = 0.0;

        // Reverse to contact then rotate the capture side toward the particle
        if (canDecontaminate(
            currentRobot,
            particle)) {
            result.captured = true;
            result.eta = 0.0;
            result.captureTime = 0.0;

            return result;
        }

        if (constraint.maxBackwardSpeed <= EPS) {
            return result;
        }

        const double particleAngle =
                angleToParticleDeg(
                    currentRobot,
                    particle
                );

        const double rearTowardParticleAngle =
                normalizeDeg(
                    particleAngle + SEMI_TURN_DEG
                );

        if (!appendRotationToAngle(
            result,
            currentRobot,
            currentTime,
            rearTowardParticleAngle,
            constraint)) {
            return makeInvalidTrajectory(
                robot,
                particle
            );
        }

        const double remainingDistance =
                distanceToContact(
                    currentRobot,
                    particle
                );

        if (remainingDistance > EPS) {
            const Time travelTime =
                    remainingDistance
                    / constraint.maxBackwardSpeed;

            appendStep(
                result,
                currentRobot,
                currentTime,
                makeBackwardCommand(constraint),
                travelTime
            );
        }

        currentRobot.leftSpeed = 0.0;
        currentRobot.rightSpeed = 0.0;

        if (canDecontaminate(
            currentRobot,
            particle)) {
            result.captured = true;
            result.eta = currentTime;
            result.captureTime = currentTime;

            return result;
        }

        waitUntilNextCommandTick(
            result,
            currentRobot,
            currentTime,
            constraint
        );

        const double mouthTowardParticleAngle =
                angleToParticleDeg(
                    currentRobot,
                    particle
                );

        if (!appendRotationToAngle(
            result,
            currentRobot,
            currentTime,
            mouthTowardParticleAngle,
            constraint)) {
            return makeInvalidTrajectory(
                robot,
                particle
            );
        }

        if (canDecontaminate(
            currentRobot,
            particle)) {
            result.captured = true;
            result.eta = currentTime;
            result.captureTime = currentTime;
        }

        return result;
    }
}


std::vector<WheelCommand> getSimpleCommands(
    const Constraint &constraint) {
    return {
        makeForwardCommand(constraint),
        makeBackwardCommand(constraint),
        makeLeftRotationCommand(constraint),
        makeRightRotationCommand(constraint),
        makeStopCommand()
    };
}


PredictedTrajectory getSimpleTrajectory(
    const RobotInfo &robot,
    const ParticleInfo &particle,
    const Constraint &constraint) {
    const PredictedTrajectory forward =
            buildForwardBaseline(
                robot,
                particle,
                constraint
            );

    const PredictedTrajectory backward =
            buildBackwardBaseline(
                robot,
                particle,
                constraint
            );

    if (!forward.captured) {
        return backward;
    }


    return backward.eta < forward.eta
               ? backward
               : forward;
}
