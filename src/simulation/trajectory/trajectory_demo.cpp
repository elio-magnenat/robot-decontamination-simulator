//
// Created by eliom on 27.08.2026.
//

#include "trajectory_simulator.h"
#include "trajectory_timeline.h"
#include "io/io.h"

#include <iostream>

int main() {
    State initialState;
    initialState.time = 0.0;
    initialState.worldOrigin.x = -150.0;
    initialState.worldOrigin.y = -75.0;
    initialState.worldEnd.x = 50.0;
    initialState.worldEnd.y = 75.0;

    RobotInfo robot;
    robot.id = 1;
    robot.position.x = 0.0;
    robot.position.y = 0.0;
    robot.radius = 10.0;
    robot.angle = 0.0;
    robot.captureAngle = 30.0;
    robot.leftSpeed = 0.0;
    robot.rightSpeed = 0.0;
    robot.score = 0.0;

    ParticleInfo particle;
    particle.id = 2;
    particle.position.x = -100.0;
    particle.position.y = 0.0;
    particle.radius = 5.0;
    particle.explosionTimes = {};

    initialState.robots.push_back(robot);
    initialState.particles.push_back(particle);

    Constraint constraint;
    constraint.CommandTimeInterval = 2.0;
    constraint.maxForwardSpeed = 20.0;
    constraint.maxBackwardSpeed = 10.0;

    WorldBounds world;
    world.origin = initialState.worldOrigin;
    world.end = initialState.worldEnd;

    const PredictedTrajectory prediction = predictTrajectory(robot,particle,constraint,world,0.0);

    if (!prediction.captured) {
        std::cout << "Demo trajectory could not capture the particle." << '\n';
        return 1;
    }

    TimeLine timeline = trajectoryToTimeline(initialState,prediction);

    const int result = exportTimeline("trajectory_demo.tlin",timeline);

    if (result != ERROR::SUCCESS) {
        std::cout << "Failed to export trajectory_demo.tlin" << '\n';
        return 1;
    }

    std::cout << "Trajectory demo generated successfully." << '\n';
    std::cout << "Captured: " << prediction.captured << '\n';
    std::cout << "ETA: " << prediction.eta << " seconds" << '\n';
    std::cout << "Steps: " << prediction.steps.size() << '\n';

    return 0;
}