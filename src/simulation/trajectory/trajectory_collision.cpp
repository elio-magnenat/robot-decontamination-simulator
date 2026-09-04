#include "trajectory_collision.h"
#include "common/Movement.h"
#include "common/Constants.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

static RobotInfo simulateCommand(const RobotInfo& robot,const WheelCommand& command,Time duration) {
    RobotInfo simulatedRobot = robot;
    simulatedRobot.leftSpeed = command.leftSpeed;
    simulatedRobot.rightSpeed = command.rightSpeed;

    return getRobotAfterTime(simulatedRobot,duration);
}

static RobotInfo stopRobot(RobotInfo robot) {
    robot.leftSpeed = 0.0;
    robot.rightSpeed = 0.0;

    return robot;
}

static double distanceToContact(const RobotInfo& robot,const ParticleInfo& particle) {
    const double dx = particle.position.x - robot.position.x;
    const double dy = particle.position.y - robot.position.y;
    const double centerDistance = std::hypot(dx,dy);

    return centerDistance - (robot.radius + particle.radius);
}

//Correct small numerical errors by placing the robot exactly at contact
static RobotInfo snapToContact(const RobotInfo& robot,const ParticleInfo& particle) {
    RobotInfo snappedRobot = robot;

    const double dx = particle.position.x - robot.position.x;
    const double dy = particle.position.y - robot.position.y;
    const double centerDistance = std::hypot(dx,dy);

    if (centerDistance <= 0.0) {
        return snappedRobot;
    }

    const double contactDistance = robot.radius + particle.radius;

    snappedRobot.position.x = particle.position.x - dx / centerDistance * contactDistance;
    snappedRobot.position.y = particle.position.y - dy / centerDistance * contactDistance;

    return snappedRobot;
}

enum class WorldBoundary {
    Left,
    Right,
    Bottom,
    Top
};

static double distanceToWorldBoundary(const RobotInfo& robot,const WorldBounds& world,WorldBoundary boundary) {
    switch (boundary) {
        case WorldBoundary::Left:
            return robot.position.x - (world.origin.x + robot.radius);
        case WorldBoundary::Right:
            return (world.end.x - robot.radius) - robot.position.x;
        case WorldBoundary::Bottom:
            return robot.position.y - (world.origin.y + robot.radius);
        case WorldBoundary::Top:
            return (world.end.y - robot.radius) - robot.position.y;
    }

    return std::numeric_limits<double>::infinity();
}

static double minimumWorldMargin(const RobotInfo& robot,const WorldBounds& world) {
    const double horizontalMargin = std::min(distanceToWorldBoundary(robot,world,WorldBoundary::Left),distanceToWorldBoundary(robot,world,WorldBoundary::Right));
    const double verticalMargin = std::min(distanceToWorldBoundary(robot,world,WorldBoundary::Bottom),distanceToWorldBoundary(robot,world,WorldBoundary::Top));

    return std::min(horizontalMargin,verticalMargin);
}

static Time findMinimumWorldBoundaryTime(const RobotInfo& robot,const WheelCommand& command,const WorldBounds& world,WorldBoundary boundary,Time low,Time high) {
    for (int iteration = 0;iteration < WORLD_MINIMUM_ITERATIONS;++iteration) {
        const Time firstThird = low + (high - low) / 3.0;
        const Time secondThird = high - (high - low) / 3.0;

        const double firstDistance = distanceToWorldBoundary(simulateCommand(robot,command,firstThird),world,boundary);
        const double secondDistance = distanceToWorldBoundary(simulateCommand(robot,command,secondThird),world,boundary);

        if (firstDistance < secondDistance) {
            high = secondThird;
        }
        else {
            low = firstThird;
        }
    }

    return (low + high) / 2.0;
}

static Time findFirstWorldCollisionTime(const RobotInfo& robot,const WheelCommand& command,const WorldBounds& world,WorldBoundary boundary,Time low,Time high) {
    const Time tolerance = std::numeric_limits<float>::epsilon();

    while (high - low > tolerance) {
        const Time middle = (low + high) / 2.0;
        const RobotInfo middleRobot = simulateCommand(robot,command,middle);

        if (distanceToWorldBoundary(middleRobot,world,boundary) < 0.0) {
            high = middle;
        }
        else {
            low = middle;
        }
    }

    return low;
}

//Search every wall because an arc can leave the world between two samples
static std::optional<Time> findWorldCollisionTimeDuringStep(const RobotInfo& robot,const WheelCommand& command,const WorldBounds& world,Time duration) {
    if (minimumWorldMargin(robot,world) < 0.0) {
        return 0.0;
    }

    Time previousTime = 0.0;
    RobotInfo previousRobot = robot;

    const Speed maxWheelSpeed = std::max(std::abs(command.leftSpeed),std::abs(command.rightSpeed));

    for (int i = 1;i <= WORLD_SAMPLE_COUNT;++i) {
        const Time currentTime = duration * i / static_cast<double>(WORLD_SAMPLE_COUNT);
        const Time intervalDuration = currentTime - previousTime;
        const double maxTravelDistance = maxWheelSpeed * intervalDuration;

        std::optional<Time> earliestCollision;

        for (int boundaryIndex = 0;boundaryIndex < 4;++boundaryIndex) {
            const WorldBoundary boundary = static_cast<WorldBoundary>(boundaryIndex);
            const double previousDistance = distanceToWorldBoundary(previousRobot,world,boundary);

            if (previousDistance > maxTravelDistance + EPS) {
                continue;
            }

            const Time minimumTime = findMinimumWorldBoundaryTime(robot,command,world,boundary,previousTime,currentTime);
            const RobotInfo minimumRobot = simulateCommand(robot,command,minimumTime);

            if (distanceToWorldBoundary(minimumRobot,world,boundary) < 0.0) {
                const Time collisionTime = findFirstWorldCollisionTime(robot,command,world,boundary,previousTime,minimumTime);

                if (!earliestCollision.has_value() || collisionTime < *earliestCollision) {
                    earliestCollision = collisionTime;
                }
            }
        }

        if (earliestCollision.has_value()) {
            return earliestCollision;
        }

        previousTime = currentTime;
        previousRobot = simulateCommand(robot,command,currentTime);
    }

    return std::nullopt;
}

static Time findMinimumDistanceTime(const RobotInfo& robot,const WheelCommand& command,const ParticleInfo& particle,Time low,Time high) {
    for (int iteration = 0;iteration < CONTACT_MINIMUM_ITERATIONS;++iteration) {
        const Time firstThird = low + (high - low) / 3.0;
        const Time secondThird = high - (high - low) / 3.0;

        const double firstDistance = distanceToContact(simulateCommand(robot,command,firstThird),particle);
        const double secondDistance = distanceToContact(simulateCommand(robot,command,secondThird),particle);

        if (firstDistance < secondDistance) {
            high = secondThird;
        }
        else {
            low = firstThird;
        }
    }

    return (low + high) / 2.0;
}

static Time findFirstContactTime(const RobotInfo& robot,const WheelCommand& command,const ParticleInfo& particle,Time low,Time high) {
    const Time tolerance = std::numeric_limits<float>::epsilon();

    while (high - low > tolerance) {
        const Time middle = (low + high) / 2.0;
        const RobotInfo middleRobot = simulateCommand(robot,command,middle);

        if (distanceToContact(middleRobot,particle) <= 0.0) {
            high = middle;
        }
        else {
            low = middle;
        }
    }

    return low;
}

//Sampling finds the interval and the searches above refine the exact contact time
static std::optional<Time> findContactTimeDuringStep(const RobotInfo& robot,const WheelCommand& command,const ParticleInfo& particle,Time duration) {
    const double initialDistance = distanceToContact(robot,particle);

    if (initialDistance <= CONTACT_TOLERANCE) {
        return 0.0;
    }

    Time previousTime = 0.0;
    double previousDistance = initialDistance;

    const Speed maxWheelSpeed = std::max(std::abs(command.leftSpeed),std::abs(command.rightSpeed));

    for (int i = 1;i <= CONTACT_SAMPLE_COUNT;++i) {
        const Time currentTime = duration * i / static_cast<double>(CONTACT_SAMPLE_COUNT);
        const RobotInfo currentRobot = simulateCommand(robot,command,currentTime);
        const double currentDistance = distanceToContact(currentRobot,particle);

        if (currentDistance <= 0.0) {
            return findFirstContactTime(robot,command,particle,previousTime,currentTime);
        }

        const Time intervalDuration = currentTime - previousTime;
        const double maxTravelDistance = maxWheelSpeed * intervalDuration;

        if (previousDistance <= maxTravelDistance + EPS) {
            const Time minimumTime = findMinimumDistanceTime(robot,command,particle,previousTime,currentTime);
            const RobotInfo minimumRobot = simulateCommand(robot,command,minimumTime);

            if (distanceToContact(minimumRobot,particle) <= 0.0) {
                return findFirstContactTime(robot,command,particle,previousTime,minimumTime);
            }
        }

        previousTime = currentTime;
        previousDistance = currentDistance;
    }

    return std::nullopt;
}

//canDecontaminate requires exact tangency so snap the robot to the contact position before checking the capture condition
static bool canDecontaminateAtContact(const RobotInfo& robot,const ParticleInfo& particle) {
    if (distanceToContact(robot,particle) > CONTACT_TOLERANCE) {
        return false;
    }

    return canDecontaminate(snapToContact(robot,particle),particle);
}

static Time findFirstCaptureTime(const RobotInfo& robot,const WheelCommand& command,const ParticleInfo& particle,Time low,Time high) {
    const Time tolerance = std::numeric_limits<float>::epsilon();

    while (high - low > tolerance) {
        const Time middle = (low + high) / 2.0;
        const RobotInfo middleRobot = simulateCommand(robot,command,middle);

        if (canDecontaminateAtContact(middleRobot,particle)) {
            high = middle;
        }
        else {
            low = middle;
        }
    }

    return high;
}

static std::optional<Time> findCaptureTimeDuringStep(const RobotInfo& robot,const WheelCommand& command,const ParticleInfo& particle,Time duration,const std::optional<Time>& contactTime) {
    if (contactTime.has_value() && *contactTime == 0.0) {
        if (canDecontaminateAtContact(robot,particle)) {
            return 0.0;
        }

        Time previousTime = 0.0;

        for (int i = 1;i <= CAPTURE_SAMPLE_COUNT;++i) {
            const Time time = duration * i / static_cast<double>(CAPTURE_SAMPLE_COUNT);
            const RobotInfo robotAtTime = simulateCommand(robot,command,time);

            if (canDecontaminateAtContact(robotAtTime,particle)) {
                return findFirstCaptureTime(robot,command,particle,previousTime,time);
            }

            previousTime = time;
        }

        return std::nullopt;
    }

    if (!contactTime.has_value()) {
        return std::nullopt;
    }

    RobotInfo robotAtContact = simulateCommand(robot,command,*contactTime);
    robotAtContact = snapToContact(robotAtContact,particle);

    if (canDecontaminate(robotAtContact,particle)) {
        return *contactTime;
    }

    return std::nullopt;
}

static bool isRotationInPlace(const WheelCommand& command) {
    return std::abs(command.leftSpeed + command.rightSpeed) < EPS;
}

static bool movesIntoParticleAtStart(const RobotInfo& robot,const WheelCommand& command,const ParticleInfo& particle,Time duration) {
    const Time probeTime = duration * CONTACT_PROBE_RATIO;
    const double beforeDistance = distanceToContact(robot,particle);
    const RobotInfo robotAfterProbe = simulateCommand(robot,command,probeTime);
    const double afterDistance = distanceToContact(robotAfterProbe,particle);

    return afterDistance < beforeDistance - CONTACT_PROBE_EPSILON;
}

//A command can first move away from a particle and come back to it later
static std::optional<Time> findRecontactTimeDuringStep(const RobotInfo& robot,const WheelCommand& command,const ParticleInfo& particle,Time duration) {
    const Time probeTime = duration * CONTACT_PROBE_RATIO;

    if (probeTime <= 0.0 || probeTime >= duration) {
        return std::nullopt;
    }

    const RobotInfo robotAfterProbe = simulateCommand(robot,command,probeTime);

    if (distanceToContact(robotAfterProbe,particle) <= 0.0) {
        return std::nullopt;
    }

    const std::optional<Time> recontactAfterProbe = findContactTimeDuringStep(robotAfterProbe,command,particle,duration - probeTime);

    if (!recontactAfterProbe.has_value()) {
        return std::nullopt;
    }

    return probeTime + *recontactAfterProbe;
}

SimulatedCandidate simulateCandidate(const RobotInfo& robot,const WheelCommand& command,const ParticleInfo& particle,const WorldBounds& world,Time duration) {
    SimulatedCandidate candidate;
    candidate.command = command;

    //limit the movement to the first world border collision
    //and search for a contact or capture of the target particle within this time
    const std::optional<Time> worldCollisionOffset = findWorldCollisionTimeDuringStep(robot,command,world,duration);
    const Time allowedDuration = worldCollisionOffset.value_or(duration);

    candidate.contactOffset = findContactTimeDuringStep(robot,command,particle,allowedDuration);
    candidate.captureOffset = findCaptureTimeDuringStep(robot,command,particle,allowedDuration,candidate.contactOffset);

    //Stop the simulation at the capture time and place the robot precisely in contact with the particle
    if (candidate.captureOffset.has_value()) {
        candidate.endRobot = simulateCommand(robot,command,*candidate.captureOffset);
        candidate.endRobot = snapToContact(candidate.endRobot,particle);
        candidate.result = CommandStepResult::Captured;
        return candidate;
    }

    // Handle if the robot is already touching the particle at the start of the command
    if (candidate.contactOffset.has_value() && *candidate.contactOffset == 0.0) {
        if (isRotationInPlace(command)) {
            candidate.endRobot = simulateCommand(robot,command,duration);
            candidate.result = CommandStepResult::Completed;
            return candidate;
        }

        if (movesIntoParticleAtStart(robot,command,particle,allowedDuration)) {
            candidate.endRobot = stopRobot(robot);
            candidate.collisionOffset = 0.0;
            candidate.result = CommandStepResult::Collided;
            return candidate;
        }

        const std::optional<Time> recontactOffset = findRecontactTimeDuringStep(robot,command,particle,allowedDuration);

        if (recontactOffset.has_value()) {
            candidate.contactOffset = recontactOffset;
            candidate.captureOffset = findCaptureTimeDuringStep(robot,command,particle,allowedDuration,candidate.contactOffset);

            if (candidate.captureOffset.has_value()) {
                candidate.endRobot = simulateCommand(robot,command,*candidate.captureOffset);
                candidate.endRobot = snapToContact(candidate.endRobot,particle);
                candidate.result = CommandStepResult::Captured;
                return candidate;
            }

            candidate.endRobot = simulateCommand(robot,command,*candidate.contactOffset);
            candidate.endRobot = snapToContact(candidate.endRobot,particle);
            candidate.endRobot = stopRobot(candidate.endRobot);
            candidate.collisionOffset = candidate.contactOffset;
            candidate.result = CommandStepResult::Collided;
            return candidate;
        }

        if (worldCollisionOffset.has_value()) {
            candidate.endRobot = simulateCommand(robot,command,*worldCollisionOffset);
            candidate.endRobot = stopRobot(candidate.endRobot);
            candidate.collisionOffset = worldCollisionOffset;
            candidate.result = CommandStepResult::Collided;
            return candidate;
        }

        candidate.endRobot = simulateCommand(robot,command,duration);
        candidate.result = CommandStepResult::Completed;
        return candidate;
    }

    //Stop the robot at the first contact with the particle if there is no capture
    if (candidate.contactOffset.has_value()) {
        candidate.endRobot = simulateCommand(robot,command,*candidate.contactOffset);
        candidate.endRobot = stopRobot(candidate.endRobot);
        candidate.collisionOffset = candidate.contactOffset;
        candidate.result = CommandStepResult::Collided;
        return candidate;
    }

    // Stop the robot when it reaches a world boundary
    if (worldCollisionOffset.has_value()) {
        candidate.endRobot = simulateCommand(robot,command,*worldCollisionOffset);
        candidate.endRobot.leftSpeed = 0.0;
        candidate.endRobot.rightSpeed = 0.0;
        candidate.collisionOffset = worldCollisionOffset;
        candidate.result = CommandStepResult::Collided;
        return candidate;
    }
    // Complete the full movement if no collision or capture occurs
    candidate.endRobot = simulateCommand(robot,command,duration);
    candidate.result = CommandStepResult::Completed;
    return candidate;
}