#ifndef PIN_2026_ALPINE_TRAJECTORY_SIMULATOR_H
#define PIN_2026_ALPINE_TRAJECTORY_SIMULATOR_H

#include "trajectory.h"
#include "common/Constraint.h"
#include <optional>
#include <vector>

enum class CommandStepResult {
    Completed,
    Captured,
    Collided
};

struct WorldBounds {
    Position origin;
    Position end;
};

struct SimulatedCandidate {
    WheelCommand command;
    RobotInfo endRobot;
    CommandStepResult result;
    std::optional<Time> contactOffset;
    std::optional<Time> captureOffset;
    std::optional<Time> collisionOffset;
};

//Predict the robot trajectory to a particle by selecting and simulating a movement at each command interval
PredictedTrajectory predictTrajectory(const RobotInfo& robot,const ParticleInfo& particle,const Constraint& constraint,const WorldBounds& world,Time startTime);

//Simulates all candidate wheel commands for a robot and returns the result of each movement
std::vector<SimulatedCandidate> simulateCandidates(const RobotInfo& robot,const std::vector<WheelCommand>& commands,const ParticleInfo& particle,const WorldBounds& world,Time duration);

#endif //PIN_2026_ALPINE_TRAJECTORY_SIMULATOR_H