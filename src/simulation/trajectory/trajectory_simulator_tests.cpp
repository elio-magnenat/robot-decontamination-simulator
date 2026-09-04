#include "trajectory_simulator.h"

#include <cassert>
#include <cmath>
#include <iostream>

static void printTrajectory(const PredictedTrajectory& trajectory) {
    std::cout << "Captured: " << trajectory.captured << '\n';
    std::cout << "Steps: " << trajectory.steps.size() << '\n';
    std::cout << "ETA: " << trajectory.eta << '\n';

    for (size_t i = 0; i < trajectory.steps.size(); ++i) {
        const TrajectoryStep& step = trajectory.steps[i];

        std::cout << "Step " << i
                  << " | left=" << step.command.leftSpeed
                  << " right=" << step.command.rightSpeed
                  << " | start=(" << step.startRobot.position.x << "," << step.startRobot.position.y << ")"
                  << " angle=" << step.startRobot.angle
                  << " | end=(" << step.endRobot.position.x << "," << step.endRobot.position.y << ")"
                  << " angle=" << step.endRobot.angle
                  << '\n';
    }
}

int main() {
    WorldBounds world;
    world.origin.x = -1000.0;
    world.origin.y = -1000.0;
    world.end.x = 1000.0;
    world.end.y = 1000.0;
    // TEST 1 : particle directly in front of the robot
    {
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
        particle.position.x = 100.0;
        particle.position.y = 0.0;
        particle.radius = 5.0;
        particle.explosionTimes = {};

        Constraint constraint;
        constraint.CommandTimeInterval = 1.0;
        constraint.maxForwardSpeed = 20.0;
        constraint.maxBackwardSpeed = 10.0;

        const PredictedTrajectory trajectory = predictTrajectory(robot,particle,constraint,world,0.0);

        std::cout << "===== STRAIGHT TEST =====" << '\n';
        printTrajectory(trajectory);

        assert(trajectory.captured);
        assert(!trajectory.steps.empty());
        assert(std::isfinite(trajectory.eta));
        assert(std::abs(trajectory.eta - 4.25) < 1e-6);

        std::cout << "Straight trajectory test passed." << '\n';
        std::cout << '\n';
    }

    // TEST 2 : particle at 90 degrees from the robot
    {
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
        particle.position.x = 0.0;
        particle.position.y = 100.0;
        particle.radius = 5.0;
        particle.explosionTimes = {};

        Constraint constraint;
        constraint.CommandTimeInterval = 1.0;
        constraint.maxForwardSpeed = 20.0;
        constraint.maxBackwardSpeed = 10.0;

        const PredictedTrajectory trajectory = predictTrajectory(robot,particle,constraint,world,0.0);

        std::cout << "===== TURN TEST =====" << '\n';
        printTrajectory(trajectory);

        assert(trajectory.captured);
        assert(!trajectory.steps.empty());
        assert(std::isfinite(trajectory.eta));
        assert(trajectory.eta > 0.0);

        std::cout << "Turn trajectory test passed." << '\n';
        std::cout << '\n';
    }

    // TEST 3 : particle directly behind the robot
    {
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

        Constraint constraint;
        constraint.CommandTimeInterval = 1.0;
        constraint.maxForwardSpeed = 20.0;
        constraint.maxBackwardSpeed = 10.0;

        const PredictedTrajectory trajectory = predictTrajectory(robot,particle,constraint,world,0.0);

        std::cout << "===== BEHIND TEST =====" << '\n';
        printTrajectory(trajectory);

        assert(trajectory.captured);
        assert(!trajectory.steps.empty());
        assert(std::isfinite(trajectory.eta));
        assert(trajectory.eta > 0.0);

        std::cout << "Behind trajectory test passed." << '\n';
        std::cout << '\n';
    }

    // TEST 4 : robot already touches the particle but is badly oriented
    {
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
        particle.position.x = 0.0;
        particle.position.y = 15.0;
        particle.radius = 5.0;
        particle.explosionTimes = {};

        Constraint constraint;
        constraint.CommandTimeInterval = 1.0;
        constraint.maxForwardSpeed = 20.0;
        constraint.maxBackwardSpeed = 10.0;

        const PredictedTrajectory trajectory = predictTrajectory(robot,particle,constraint,world,0.0);

        std::cout << "===== CONTACT ROTATION TEST =====" << '\n';
        printTrajectory(trajectory);

        assert(trajectory.captured);
        assert(!trajectory.steps.empty());
        assert(std::isfinite(trajectory.eta));
        assert(trajectory.eta > 0.0);

        std::cout << "Contact rotation trajectory test passed." << '\n';
        std::cout << '\n';
    }

    // TEST 5 : robot already touches the particle and is correctly oriented
    {
        RobotInfo robot;
        robot.id = 1;
        robot.position.x = 0.0;
        robot.position.y = 0.0;
        robot.radius = 10.0;
        robot.angle = 90.0;
        robot.captureAngle = 30.0;
        robot.leftSpeed = 0.0;
        robot.rightSpeed = 0.0;
        robot.score = 0.0;

        ParticleInfo particle;
        particle.id = 2;
        particle.position.x = 0.0;
        particle.position.y = 15.0;
        particle.radius = 5.0;
        particle.explosionTimes = {};

        Constraint constraint;
        constraint.CommandTimeInterval = 1.0;
        constraint.maxForwardSpeed = 20.0;
        constraint.maxBackwardSpeed = 10.0;

        const PredictedTrajectory trajectory = predictTrajectory(robot,particle,constraint,world,0.0);

        std::cout << "===== IMMEDIATE CAPTURE TEST =====" << '\n';
        printTrajectory(trajectory);

        assert(trajectory.captured);
        assert(trajectory.steps.empty());
        assert(std::isfinite(trajectory.eta));
        assert(std::abs(trajectory.eta) < 1e-9);
        assert(std::abs(trajectory.captureTime) < 1e-9);

        std::cout << "Immediate capture trajectory test passed." << '\n';
        std::cout << '\n';
    }

    // TEST 6 : contact happens entirely between two samples
    {
        RobotInfo robot;
        robot.id = 1;
        robot.position.x = 0.0;
        robot.position.y = 0.0;
        robot.radius = 0.1;
        robot.angle = 0.0;
        robot.captureAngle = 30.0;
        robot.leftSpeed = 0.0;
        robot.rightSpeed = 0.0;
        robot.score = 0.0;

        ParticleInfo particle;
        particle.id = 2;
        particle.position.x = 1.0;
        particle.position.y = 0.0;
        particle.radius = 0.1;
        particle.explosionTimes = {};

        WheelCommand command;
        command.leftSpeed = 100.0;
        command.rightSpeed = 100.0;

        const std::vector<WheelCommand> commands = {command};
        const std::vector<SimulatedCandidate> candidates = simulateCandidates(robot,commands,particle,world,1.0);

        std::cout << "===== BRIEF CONTACT TEST =====" << '\n';

        assert(candidates.size() == 1);

        const SimulatedCandidate& candidate = candidates.front();

        assert(candidate.result == CommandStepResult::Captured);
        assert(candidate.contactOffset.has_value());
        assert(candidate.captureOffset.has_value());
        assert(*candidate.contactOffset > 0.0);
        assert(*candidate.contactOffset < 0.02);
        assert(std::abs(*candidate.contactOffset - 0.008) < 1e-5);

        std::cout << "Contact time: " << *candidate.contactOffset << '\n';
        std::cout << "Brief contact test passed." << '\n';
        std::cout << '\n';
    }

    // TEST 7 : robot must stop at the world boundary
    {
        RobotInfo robot;
        robot.id = 1;
        robot.position.x = 85.0;
        robot.position.y = 0.0;
        robot.radius = 10.0;
        robot.angle = 0.0;
        robot.captureAngle = 30.0;
        robot.leftSpeed = 0.0;
        robot.rightSpeed = 0.0;
        robot.score = 0.0;

        ParticleInfo particle;
        particle.id = 2;
        particle.position.x = 0.0;
        particle.position.y = 80.0;
        particle.radius = 1.0;
        particle.explosionTimes = {};

        WorldBounds boundaryWorld;
        boundaryWorld.origin.x = -100.0;
        boundaryWorld.origin.y = -100.0;
        boundaryWorld.end.x = 100.0;
        boundaryWorld.end.y = 100.0;

        WheelCommand command;
        command.leftSpeed = 20.0;
        command.rightSpeed = 20.0;

        const std::vector<WheelCommand> commands = {command};
        const std::vector<SimulatedCandidate> candidates = simulateCandidates(robot,commands,particle,boundaryWorld,1.0);

        std::cout << "===== WORLD BOUNDARY TEST =====" << '\n';

        assert(candidates.size() == 1);

        const SimulatedCandidate& candidate = candidates.front();

        assert(candidate.result == CommandStepResult::Collided);
        assert(candidate.collisionOffset.has_value());
        assert(std::abs(*candidate.collisionOffset - 0.25) < 1e-5);
        assert(candidate.endRobot.position.x <= 90.0 + 1e-5);
        assert(std::abs(candidate.endRobot.leftSpeed) < 1e-9);
        assert(std::abs(candidate.endRobot.rightSpeed) < 1e-9);

        std::cout << "Boundary collision time: " << *candidate.collisionOffset << '\n';
        std::cout << "World boundary test passed." << '\n';
        std::cout << '\n';
    }
    // TEST 8 : with frequent commands, the fixed-depth search must still value
    // the faster arc instead of optimizing only the next few tenths of a second.
    {
        RobotInfo robot;
        robot.id = 1;
        robot.position.x = 195.0;
        robot.position.y = 112.5;
        robot.radius = 20.0;
        robot.angle = 30.0;
        robot.captureAngle = 10.0;
        robot.leftSpeed = 0.0;
        robot.rightSpeed = 0.0;
        robot.score = 0.0;

        ParticleInfo particle;
        particle.id = 5;
        particle.position.x = 443.0;
        particle.position.y = 78.0;
        particle.radius = 40.0;

        Constraint constraint;
        constraint.CommandTimeInterval = 0.25;
        constraint.maxForwardSpeed = 20.0;
        constraint.maxBackwardSpeed = 5.0;

        const PredictedTrajectory trajectory = predictTrajectory(robot, particle, constraint, world, 0.0);

        std::cout << "===== FAST COMMAND LOOKAHEAD TEST =====" << '\n';
        printTrajectory(trajectory);

        assert(trajectory.captured);
        assert(!trajectory.steps.empty());
        assert(trajectory.eta < 13.0);
        assert(std::abs(trajectory.steps.front().command.leftSpeed
                      - trajectory.steps.front().command.rightSpeed) > 1e-9);

        std::cout << "Fast command lookahead test passed." << '\n';
        std::cout << '\n';
    }
    std::cout << "All trajectory simulator tests passed." << '\n';
    std::cout << '\n';

    return 0;
}