#ifndef PIN_2026_ALPINE_TRAJECTORY_H
#define PIN_2026_ALPINE_TRAJECTORY_H

#include <vector>
#include <limits>

#include <common/Robot.h>
#include <common/Particle.h>

//Associates a robot with its target particle
struct Trajectory {
    RobotInfo robot;
    ParticleInfo particle;
};

//Stores the left and right wheel speeds of a robot command
struct WheelCommand {
    Speed leftSpeed;
    Speed rightSpeed;
};

//Represents one movement step of a predicted trajectory including its timing, command, and robot states
struct TrajectoryStep {
    Time startTime;
    Time duration;
    WheelCommand command;
    RobotInfo startRobot;
    RobotInfo endRobot;
};

//Stores the complete predicted trajectory of a robot with a target particle, including its movement steps and capture information
struct PredictedTrajectory {
    UniqueId robotId;
    UniqueId particleId;
    bool captured = false;
    Time eta = std::numeric_limits<Time>::infinity();
    Time captureTime = std::numeric_limits<Time>::infinity();
    std::vector<TrajectoryStep> steps;
};

#endif // PIN_2026_ALPINE_TRAJECTORY_H