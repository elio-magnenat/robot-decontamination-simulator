#include "trajectory_planner.h"
#include "trajectory_collision.h"
#include "simple_trajectory.h"
#include "arc_trajectory.h"
#include "trajectory_score.h"
#include "common/Constants.h"
#include "common/angle_utils.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

struct ScoredCommand {
    SimulatedCandidate candidate;
    double score;
};

//Build the list of wheel commands that can be evaluated by the trajectory planner
static std::vector<WheelCommand> getCandidateCommands(const Constraint& constraint) {
    std::vector<WheelCommand> commands = getSimpleCommands(constraint);
    const std::vector<WheelCommand> arcCommands = getArcCommands(constraint);

    commands.insert(commands.end(), arcCommands.begin(), arcCommands.end());

    return commands;
}

//Scores a simulated command according to its movement quality, rewarding fast captures and penalizing collisions
static double scoreCandidate(const RobotInfo& robot, const ParticleInfo& particle, const Constraint& constraint,const SimulatedCandidate& candidate) {
    EvalContext context;
    context.constraint = constraint;

    double score = evaluateTrajectoryStep(robot, candidate.endRobot, particle, candidate.command, context);

    if (candidate.result == CommandStepResult::Captured) {
        const Time captureTime = candidate.captureOffset.value_or(constraint.CommandTimeInterval);
        score += context.weights.capture - captureTime;
    }
    else if (candidate.result == CommandStepResult::Collided) {
        //A collision with a wall is never a valid planning choice
        if (!candidate.contactOffset.has_value()) {
            return -std::numeric_limits<double>::infinity();
        }

        score -= context.weights.blocked;
    }

    return score;
}

//Select the weight used for the remaining-time estimate according to the movement constraints
static double terminalEtaWeight(const Constraint& constraint) {
    if (constraint.CommandTimeInterval <= FAST_COMMAND_INTERVAL_THRESHOLD
        && constraint.maxForwardSpeed > constraint.maxBackwardSpeed) {
        return FAST_COMMAND_TERMINAL_ETA_WEIGHT;
    }

    return TERMINAL_ETA_WEIGHT;
}

double estimateRemainingTime(const RobotInfo& robot, const ParticleInfo& particle, const Constraint& constraint) {
    //Estimate the minimum time needed to cover the remaining distance with maximum forward speed
    const double remainingDistance = gap(robot, particle);
    double translationTime = 0.0;

    if (constraint.maxForwardSpeed > 0.0) {
        translationTime = remainingDistance / constraint.maxForwardSpeed;
    }

    //Estimate the maximum speed for rotating the robot in place
    const Speed rotationWheelSpeed = std::min(constraint.maxForwardSpeed, constraint.maxBackwardSpeed);
    double angularSpeedDeg = 0.0;

    if (robot.radius > 0.0) {
        angularSpeedDeg = rotationWheelSpeed / robot.radius * RAD2DEG;
    }

    //Estimate the time needed to rotate until the target is inside the robot's capture angle
    const double remainingAngle = std::max(0.0, angleErrorDeg(robot, particle) - robot.captureAngle);

    double rotationTime = 0.0;

    if (angularSpeedDeg > 0.0) {
        rotationTime = remainingAngle / angularSpeedDeg;
    }
    // Combine the estimated translation and rotation times
    return translationTime + rotationTime;
}

static double evaluateFuture(const RobotInfo& robot, const ParticleInfo& particle, const Constraint& constraint, const std::vector<WheelCommand>& commands, const WorldBounds& world, int depth) {
    std::vector<ScoredCommand> scoredCandidates;
    double bestScore = -std::numeric_limits<double>::infinity();
    std::size_t branchCount = 0;
    std::size_t i = 0;
    //Stop the lookahead at the maximum depth and estimate the remaining time to the target as the terminal score
    if (depth <= 0) {
        return -terminalEtaWeight(constraint) * estimateRemainingTime(robot, particle, constraint);
    }

    if (commands.empty()) {
        return 0.0;
    }
    //Evaluate all commands for one step before selecting which branches are worth exploring further
    const std::vector<SimulatedCandidate> candidates = simulateCandidates(robot, commands, particle, world, constraint.CommandTimeInterval);
    scoredCandidates.reserve(candidates.size());

    for (const SimulatedCandidate& candidate : candidates) {
        const double score = scoreCandidate(robot, particle, constraint, candidate);

        //A capture ends this branch because no further movement needs to be evaluated
        if (candidate.result == CommandStepResult::Captured) {
            if (score > bestScore) {
                bestScore = score;
            }
            continue;
        }

        scoredCandidates.push_back({candidate, score});
    }

    //Sort the candidates by score and keep only the best ones for deeper exploration
    std::sort(scoredCandidates.begin(), scoredCandidates.end(),
    [](const ScoredCommand& first, const ScoredCommand& second) {
                  return first.score > second.score;
              });

    branchCount = std::min(BRANCHING_TOP_COUNT, scoredCandidates.size());

    //Explore the best branches recursively and combine their score with a score of their future movements
    for (i = 0; i < branchCount; ++i) {
        const double score = scoredCandidates[i].score + FUTURE_DISCOUNT * evaluateFuture(scoredCandidates[i].candidate.endRobot, particle, constraint, commands,world, depth - 1);

        if (score > bestScore) {
            bestScore = score;
        }
    }

    return bestScore;
}

//Clamp a wheel speed to the allowed forward and backward speed limits
static Speed clampWheelSpeed(Speed speed, const Constraint& constraint) {
    return std::clamp(speed, -constraint.maxBackwardSpeed, constraint.maxForwardSpeed);
}
// Check if two wheel commands are equivalent with a numerical tolerance
static bool sameWheelCommand(const WheelCommand& first, const WheelCommand& second) {
    return std::abs(first.leftSpeed - second.leftSpeed) < EPS && std::abs(first.rightSpeed - second.rightSpeed) < EPS;
}

//Create small variations around the commands that already scored well
static std::vector<WheelCommand> getRefinedCommands(const WheelCommand& command, const Constraint& constraint) {
    std::vector<WheelCommand> refinedCommands;

    const Speed maxSpeed = std::max(constraint.maxForwardSpeed, constraint.maxBackwardSpeed);
    const Speed refinementStep = maxSpeed * REFINEMENT_RATIO;

    if (refinementStep <= 0.0) {
        return refinedCommands;
    }

    const Speed offsets[] = {-refinementStep, 0.0, refinementStep};

    // Generate all combinations of small speed variations for both wheels
    for (const Speed leftOffset : offsets) {
        for (const Speed rightOffset : offsets) {
            if (leftOffset == 0.0 && rightOffset == 0.0) {
                continue;
            }

            WheelCommand refinedCommand;
            refinedCommand.leftSpeed = clampWheelSpeed(command.leftSpeed + leftOffset, constraint);
            refinedCommand.rightSpeed = clampWheelSpeed(command.rightSpeed + rightOffset, constraint);

            if (sameWheelCommand(refinedCommand, command)) {
                continue;
            }

            const bool alreadyAdded = std::any_of(refinedCommands.begin(), refinedCommands.end(),
                                                  [&refinedCommand](const WheelCommand& existingCommand) {
                                                      return sameWheelCommand(existingCommand, refinedCommand);
                                                  });

            if (alreadyAdded) {
                continue;
            }

            refinedCommands.push_back(refinedCommand);
        }
    }

    return refinedCommands;
}

//Evaluate one wheel command using its immediate result and its predicted future score
static ScoredCommand evaluateCommandChoice(const RobotInfo& robot, const ParticleInfo& particle,const Constraint& constraint, const std::vector<WheelCommand>& commands,const WorldBounds& world, const WheelCommand& command, Time stepDuration,int lookaheadDepth) {
    ScoredCommand result;
    result.candidate = simulateCandidate(robot, command, particle, world, stepDuration);
    result.score = scoreCandidate(robot, particle, constraint, result.candidate);

    if (result.candidate.result != CommandStepResult::Captured) {
        result.score += FUTURE_DISCOUNT * evaluateFuture(result.candidate.endRobot, particle, constraint, commands, world, lookaheadDepth - 1);
    }

    return result;
}


SimulatedCandidate chooseBestCommand(const RobotInfo& currentRobot, const ParticleInfo& particle, const Constraint& constraint, const WorldBounds& world, Time stepDuration) {
    //Stop the robot if no candidate movement command is available
    const std::vector<WheelCommand> commands = getCandidateCommands(constraint);
    if (commands.empty()) {
        WheelCommand stopCommand;
        stopCommand.leftSpeed = 0.0;
        stopCommand.rightSpeed = 0.0;

        return simulateCandidate(currentRobot, stopCommand, particle, world, stepDuration);
    }


    std::vector<ScoredCommand> scoredCommands;
    scoredCommands.reserve(commands.size());

    double bestScore = -std::numeric_limits<double>::infinity();
    SimulatedCandidate bestCandidate;

    //Evaluate all standard commands and keep track of the best candidate
    for (const WheelCommand& command : commands) {
        const ScoredCommand scoredCommand = evaluateCommandChoice(currentRobot, particle, constraint, commands, world,command, stepDuration, LOOKAHEAD_DEPTH);
        const double score = scoredCommand.score;

        scoredCommands.push_back(scoredCommand);

        if (scoredCommands.size() == 1 || score > bestScore) {
            bestScore = score;
            bestCandidate = scoredCommand.candidate;
        }
    }
    std::sort(scoredCommands.begin(), scoredCommands.end(),
              [](const ScoredCommand& first, const ScoredCommand& second) {
                  return first.score > second.score;
              });

    //Only refine the best standard commands to keep the search small
    const std::size_t commandsToRefine = std::min(REFINE_TOP_COUNT, scoredCommands.size());

    //Generate more precise command variations around the best standard candidates
    for (std::size_t i = 0; i < commandsToRefine; ++i) {
        const std::vector<WheelCommand> refinedCommands = getRefinedCommands(scoredCommands[i].candidate.command, constraint);
        //Evaluate each refined command and update the best candidate if it improves the score
        for (const WheelCommand& refinedCommand : refinedCommands) {
            const ScoredCommand scoredCommand = evaluateCommandChoice(currentRobot, particle, constraint, commands, world, refinedCommand, stepDuration,LOOKAHEAD_DEPTH);
            const double score = scoredCommand.score;

            if (score > bestScore) {
                bestScore = score;
                bestCandidate = scoredCommand.candidate;
            }
        }
    }

    return bestCandidate;
}
