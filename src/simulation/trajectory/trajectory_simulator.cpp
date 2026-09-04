#include "trajectory_simulator.h"
#include "trajectory_collision.h"
#include "trajectory_planner.h"
#include "common/Movement.h"
#include "common/Constants.h"
#include <cmath>
#include <vector>

//Add one executed command to the predicted trajectory
static void applyTrajectoryStep(PredictedTrajectory& trajectory,RobotInfo& currentRobot,Time& currentTime,const WheelCommand& command,Time duration) {
    TrajectoryStep step;
    step.startTime = currentTime;
    step.duration = duration;
    step.command = command;
    step.startRobot = currentRobot;

    RobotInfo endRobot = currentRobot;
    endRobot.leftSpeed = command.leftSpeed;
    endRobot.rightSpeed = command.rightSpeed;
    step.endRobot = getRobotAfterTime(endRobot,duration);

    trajectory.steps.push_back(step);
    currentRobot = step.endRobot;
    currentTime += duration;
}

std::vector<SimulatedCandidate> simulateCandidates(const RobotInfo& robot,const std::vector<WheelCommand>& commands,const ParticleInfo& particle,const WorldBounds& world,Time duration) {
    std::vector<SimulatedCandidate> candidates;
    candidates.reserve(commands.size());
    //Simulate each possible wheel command and store the result
    for (const WheelCommand& command : commands) {
        candidates.push_back(simulateCandidate(robot,command,particle,world,duration));
    }

    return candidates;
}

static CommandStepResult executeCommandStep(PredictedTrajectory& trajectory,RobotInfo& currentRobot,Time& currentTime,const SimulatedCandidate& candidate,Time stepDuration,Time startTime) {
    const WheelCommand& command = candidate.command;

    //Stop the movement at the capture time and update the predicted trajectory with the successful capture
    if (candidate.result == CommandStepResult::Captured) {
        const Time captureOffset = candidate.captureOffset.value_or(0.0);

        if (captureOffset > 0.0) {
            applyTrajectoryStep(trajectory,currentRobot,currentTime,command,captureOffset);
            currentRobot = candidate.endRobot;
            trajectory.steps.back().endRobot = currentRobot;
        }

        trajectory.captured = true;
        trajectory.captureTime = currentTime;
        trajectory.eta = currentTime - startTime;

        return CommandStepResult::Captured;
    }

    //Stop the robot at the collision time and keep it stopped for the end of the command interval
    if (candidate.result == CommandStepResult::Collided) {
        const Time collisionOffset = candidate.collisionOffset.value_or(0.0);

        if (collisionOffset > 0.0) {
            applyTrajectoryStep(trajectory,currentRobot,currentTime,command,collisionOffset);
            currentRobot = candidate.endRobot;
            trajectory.steps.back().endRobot = currentRobot;
        }

        const Time remainingTime = stepDuration - collisionOffset;

        if (remainingTime > 0.0) {
            WheelCommand stopCommand;
            stopCommand.leftSpeed = 0.0;
            stopCommand.rightSpeed = 0.0;

            applyTrajectoryStep(trajectory,currentRobot,currentTime,stopCommand,remainingTime);
        }

        return CommandStepResult::Collided;
    }

    //Execute the selected command for the full interval when no capture and no collision occurs
    applyTrajectoryStep(trajectory,currentRobot,currentTime,command,stepDuration);

    return CommandStepResult::Completed;
}

PredictedTrajectory predictTrajectory(const RobotInfo& robot,const ParticleInfo& particle,const Constraint& constraint,const WorldBounds& world,Time startTime) {
    PredictedTrajectory result;
    result.robotId = robot.id;
    result.particleId = particle.id;

    RobotInfo currentRobot = robot;
    Time currentTime = startTime;

    const Time stepDuration = constraint.CommandTimeInterval;

    if (stepDuration <= 0.0) {
        return result;
    }

    int stagnantSteps = 0;

    //Build the predicted trajectory step by step by selecting and executing
    //the best command at each allowed command interval
    for (int stepIndex = 0;stepIndex < MAX_STEPS;++stepIndex) {
        const RobotInfo robotBeforeStep = currentRobot;
        const SimulatedCandidate candidate = chooseBestCommand(currentRobot,particle,constraint,world,stepDuration);
        const CommandStepResult stepResult = executeCommandStep(result,currentRobot,currentTime,candidate,stepDuration,startTime);

        if (stepResult == CommandStepResult::Captured) {
            return result;
        }

        //Check if the robot really moved or rotated during this step
        const bool samePosition = std::abs(currentRobot.position.x - robotBeforeStep.position.x) < EPS && std::abs(currentRobot.position.y - robotBeforeStep.position.y) < EPS;
        const bool sameAngle = std::abs(normalizeDeg(currentRobot.angle - robotBeforeStep.angle)) < EPS;

        //Stop the prediction if the robot don't do anything for too many consecutive steps
        if (samePosition && sameAngle) {
            ++stagnantSteps;
        }
        else {
            stagnantSteps = 0;
        }

        if (stagnantSteps >= MAX_STAGNANT_STEPS) {
            break;
        }
    }

    return result;
}