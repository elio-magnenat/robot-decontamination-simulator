#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

#include "simple_trajectory.h"

namespace {

constexpr double TEST_EPS = 1e-7;

bool approximatelyEqual(
    const double a,
    const double b,
    const double epsilon = TEST_EPS)
{
    return std::abs(a - b) <= epsilon;
}

RobotInfo createTestRobot()
{
    RobotInfo robot{};

    robot.id = 0;
    robot.position = {0.0, 0.0};
    robot.radius = 20.0;
    robot.angle = 0.0;       // 0° faces right
    robot.captureAngle = 10.0;
    robot.leftSpeed = 0.0;
    robot.rightSpeed = 0.0;
    robot.score = 0.0;

    return robot;
}

ParticleInfo createTestParticle(
    const double x,
    const double y)
{
    ParticleInfo particle{};

    particle.id = 0;
    particle.position = {x, y};
    particle.radius = 10.0;

    return particle;
}

Constraint createTestConstraint(
    const double maxForwardSpeed = 20.0,
    const double maxBackwardSpeed = 5.0,
    const double commandTimeInterval = 1.0)
{
    Constraint constraint{};

    constraint.maxForwardSpeed = maxForwardSpeed;
    constraint.maxBackwardSpeed = maxBackwardSpeed;
    constraint.CommandTimeInterval = commandTimeInterval;

    return constraint;
}

void printTrajectory(
    const std::string& name,
    const PredictedTrajectory& trajectory)
{
    std::cout << "\n--- " << name << " ---\n";
    std::cout << "Captured : " << (trajectory.captured ? "oui" : "non") << '\n';
    std::cout << "ETA : " << trajectory.eta << " s\n";
    std::cout << "Nombre d'etapes : " << trajectory.steps.size() << '\n';

    for (std::size_t i = 0; i < trajectory.steps.size(); ++i) {
        const TrajectoryStep& step = trajectory.steps[i];

        std::cout
            << "Etape " << i
            << " | t = " << step.startTime
            << " | duree = " << step.duration
            << " | commande = ("
            << step.command.leftSpeed << ", "
            << step.command.rightSpeed << ")"
            << " | position = ("
            << step.startRobot.position.x << ", "
            << step.startRobot.position.y << ") -> ("
            << step.endRobot.position.x << ", "
            << step.endRobot.position.y << ")"
            << " | angle = "
            << step.startRobot.angle << " -> "
            << step.endRobot.angle
            << '\n';
    }
}

void testSimpleCommands()
{
    std::cout << "\nTest 1 : commandes simples...\n";

    // Use asymmetric limits to verify forward, backward and rotation speeds
    const Constraint constraint = createTestConstraint(
        5.0,
        20.0,
        1.0
    );

    const auto commands = getSimpleCommands(constraint);

    assert(commands.size() == 5);

    assert(approximatelyEqual(commands[0].leftSpeed, 5.0));
    assert(approximatelyEqual(commands[0].rightSpeed, 5.0));

    assert(approximatelyEqual(commands[1].leftSpeed, -20.0));
    assert(approximatelyEqual(commands[1].rightSpeed, -20.0));

    // Rotation speed is limited by the lower forward/backward limit
    assert(approximatelyEqual(commands[2].leftSpeed, -5.0));
    assert(approximatelyEqual(commands[2].rightSpeed, 5.0));

    assert(approximatelyEqual(commands[3].leftSpeed, 5.0));
    assert(approximatelyEqual(commands[3].rightSpeed, -5.0));

    assert(approximatelyEqual(commands[4].leftSpeed, 0.0));
    assert(approximatelyEqual(commands[4].rightSpeed, 0.0));

    // Every command must respect the wheel speed limits
    for (const WheelCommand& command : commands) {
        assert(command.leftSpeed <= constraint.maxForwardSpeed + TEST_EPS);
        assert(command.leftSpeed >= -constraint.maxBackwardSpeed - TEST_EPS);
        assert(command.rightSpeed <= constraint.maxForwardSpeed + TEST_EPS);
        assert(command.rightSpeed >= -constraint.maxBackwardSpeed - TEST_EPS);
    }

    std::cout << "OK\n";
}

void testTargetInFront()
{
    std::cout << "\nTest 2 : cible exactement devant...\n";

    const RobotInfo robot = createTestRobot();
    const ParticleInfo particle = createTestParticle(100.0, 0.0);
    const Constraint constraint = createTestConstraint();

    const PredictedTrajectory trajectory = getSimpleTrajectory(
        robot,
        particle,
        constraint
    );

    printTrajectory("Cible devant", trajectory);

    assert(trajectory.captured);
    assert(!trajectory.steps.empty());

    // No rotation is needed when the particle is directly ahead
    const WheelCommand& firstCommand = trajectory.steps.front().command;
    assert(firstCommand.leftSpeed > 0.0);
    assert(firstCommand.rightSpeed > 0.0);
    assert(approximatelyEqual(firstCommand.leftSpeed, firstCommand.rightSpeed));

    // Contact is reached after (100 - 20 - 10) / 20 = 3.5 s
    assert(approximatelyEqual(trajectory.eta, 3.5));
    assert(approximatelyEqual(trajectory.captureTime, 3.5));

    std::cout << "OK\n";
}

void testTargetOnLeft()
{
    std::cout << "\nTest 3 : cible a gauche...\n";

    const RobotInfo robot = createTestRobot();
    const ParticleInfo particle = createTestParticle(0.0, 100.0);
    const Constraint constraint = createTestConstraint();

    const PredictedTrajectory trajectory = getSimpleTrajectory(
        robot,
        particle,
        constraint
    );

    printTrajectory("Cible a gauche", trajectory);

    assert(trajectory.captured);
    assert(!trajectory.steps.empty());

    const WheelCommand& firstCommand = trajectory.steps.front().command;
    assert(firstCommand.leftSpeed < 0.0);
    assert(firstCommand.rightSpeed > 0.0);

    std::cout << "OK\n";
}

void testTargetOnRight()
{
    std::cout << "\nTest 4 : cible a droite...\n";

    const RobotInfo robot = createTestRobot();
    const ParticleInfo particle = createTestParticle(0.0, -100.0);
    const Constraint constraint = createTestConstraint();

    const PredictedTrajectory trajectory = getSimpleTrajectory(
        robot,
        particle,
        constraint
    );

    printTrajectory("Cible a droite", trajectory);

    assert(trajectory.captured);
    assert(!trajectory.steps.empty());

    const WheelCommand& firstCommand = trajectory.steps.front().command;
    assert(firstCommand.leftSpeed > 0.0);
    assert(firstCommand.rightSpeed < 0.0);

    std::cout << "OK\n";
}

void testTargetBehindWithFastForward()
{
    std::cout << "\nTest 5 : cible derriere, marche avant rapide...\n";

    const RobotInfo robot = createTestRobot();

    // The particle is directly behind the robot
    const ParticleInfo particle = createTestParticle(-100.0, 0.0);

    const Constraint constraint = createTestConstraint(
        20.0,
        5.0,
        1.0
    );

    const PredictedTrajectory trajectory = getSimpleTrajectory(
        robot,
        particle,
        constraint
    );

    printTrajectory("Derriere avec avant rapide", trajectory);

    assert(trajectory.captured);
    assert(!trajectory.steps.empty());

    // With faster forward motion the selected baseline should rotate then move forward
    const WheelCommand& firstCommand = trajectory.steps.front().command;
    assert(firstCommand.leftSpeed * firstCommand.rightSpeed < 0.0);

    bool forwardWasUsed = false;
    for (const TrajectoryStep& step : trajectory.steps) {
        if (step.command.leftSpeed > 0.0
            && step.command.rightSpeed > 0.0
            && approximatelyEqual(step.command.leftSpeed, step.command.rightSpeed)) {
            forwardWasUsed = true;
        }
    }

    assert(forwardWasUsed);

    std::cout << "OK\n";
}

void testFastBackward()
{
    std::cout << "\nTest 6 : cible derriere, marche arriere rapide...\n";

    const RobotInfo robot = createTestRobot();
    const ParticleInfo particle = createTestParticle(-100.0, 0.0);

    // Use a much faster reverse speed to favor the backward baseline
    const Constraint constraint = createTestConstraint(
        5.0,
        20.0,
        1.0
    );

    const PredictedTrajectory trajectory = getSimpleTrajectory(
        robot,
        particle,
        constraint
    );

    printTrajectory("Derriere avec arriere rapide", trajectory);

    assert(trajectory.captured);
    assert(!trajectory.steps.empty());

    bool backwardWasUsed = false;

    for (const TrajectoryStep& step : trajectory.steps) {
        if (step.command.leftSpeed < 0.0
            && step.command.rightSpeed < 0.0
            && approximatelyEqual(step.command.leftSpeed, step.command.rightSpeed)) {
            backwardWasUsed = true;
        }
    }

    assert(backwardWasUsed);

    std::cout << "OK\n";
}

void testCommandTimeInterval()
{
    std::cout << "\nTest 7 : respect de CommandTimeInterval...\n";

    const RobotInfo robot = createTestRobot();
    const ParticleInfo particle = createTestParticle(0.0, 100.0);

    const Constraint constraint = createTestConstraint(
        20.0,
        5.0,
        1.0
    );

    const PredictedTrajectory trajectory = getSimpleTrajectory(
        robot,
        particle,
        constraint
    );

    printTrajectory("CommandTimeInterval", trajectory);

    assert(trajectory.captured);
    assert(!trajectory.steps.empty());

    const double dt = constraint.CommandTimeInterval;
    bool rotationFound = false;

    for (const TrajectoryStep& step : trajectory.steps) {
        const WheelCommand& command = step.command;

        const bool isRotation =
            command.leftSpeed * command.rightSpeed < 0.0;

        if (!isRotation) {
            continue;
        }

        rotationFound = true;

        // Rotation duration stays on a full command interval
        assert(approximatelyEqual(step.duration, dt));

        // Rotation steps must start on valid command ticks
        const double tickNumber = step.startTime / dt;
        assert(approximatelyEqual(tickNumber, std::round(tickNumber)));
    }

    assert(rotationFound);

    std::cout << "OK\n";
}

} // namespace

int main()
{
    testSimpleCommands();
    testTargetInFront();
    testTargetOnLeft();
    testTargetOnRight();
    testTargetBehindWithFastForward();
    testFastBackward();
    testCommandTimeInterval();

    std::cout
        << "\n=================================\n"
        << "TOUS LES TESTS SONT PASSES !\n"
        << "=================================\n";

    return 0;
}