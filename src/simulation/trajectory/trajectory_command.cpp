#include "trajectory_command.h"

#include <algorithm>
#include <limits>
#include <vector>

#include "trajectory_planner.h"
#include "trajectory_simulator.h"

/**
 * @brief Borders of the field in the format wanted by the planner
 *
 * @param state : state of the simulation
 * @return the borders of the world
 */
static WorldBounds worldOf(const State& state) {
    WorldBounds world = {};

    world.origin = state.worldOrigin;
    world.end = state.worldEnd;

    return world;
}

/**
 * @brief Writes the speeds of a command into the matching robot of the state
 *
 * @param robots : list of the robots
 * @param robotId : ID of the robot to change
 * @param command : the two wheel speeds to copy
 */
static void setSpeeds(Robots& robots, const UniqueId robotId, const WheelCommand& command) {
    for (RobotInfo& robot : robots) {
        if (robot.id == robotId) {
            robot.leftSpeed = command.leftSpeed;
            robot.rightSpeed = command.rightSpeed;
            return;
        }
    }
}

/**
 * @brief The planner works on the robot as it is in the current state
 *
 * @param robots : list of the robots
 * @param robotId : ID of the robot we look for
 * @return a pointer on the robot, or nullptr if it is not found
 */
static const RobotInfo* findRobot(const Robots& robots, const UniqueId robotId) {
    for (const RobotInfo& robot : robots) {
        if (robot.id == robotId) return &robot;
    }
    return nullptr;
}

void applyPlannedCommands(State& state, const std::vector<Trajectory>& trajectories,
                          const Constraint& constraint) {
    const WorldBounds world = worldOf(state);
    const RobotInfo* current = nullptr;

    for (const Trajectory& trajectory : trajectories) {
        //We start again from the robot of the state : the commands already set in
        //this loop must be seen by the next robots.
        current = findRobot(state.robots, trajectory.robot.id);
        if (current == nullptr) continue;

        const SimulatedCandidate candidate = chooseBestCommand(*current, trajectory.particle,
                                                               constraint, world,
                                                               constraint.CommandTimeInterval);

        setSpeeds(state.robots, trajectory.robot.id, candidate.command);
    }
}

std::vector<Trajectory> findBestParticlesByEta(const State& state, const Constraint& constraint) {
    std::vector<Trajectory> best;
    std::vector<bool> taken(state.particles.size(), false);
    size_t bestIndex = 0;
    double bestCost = 0.0;
    double cost = 0.0;
    bool found = false;

    best.reserve(std::min(state.robots.size(), state.particles.size()));

    for (const RobotInfo& robot : state.robots) {
        bestCost = std::numeric_limits<double>::infinity();
        found = false;

        for (size_t i = 0; i < state.particles.size(); ++i) {
            if (taken[i]) continue;

            cost = estimateRemainingTime(robot, state.particles[i], constraint);

            if (!found || cost < bestCost) {
                bestCost = cost;
                bestIndex = i;
                found = true;
            }
        }

        if (!found) break; //no free particle left : the other robots have no target

        taken[bestIndex] = true;
        best.push_back({robot, state.particles[bestIndex]});
    }

    return best;
}
