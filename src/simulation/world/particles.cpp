//
// Created by niels on 25.08.2026.
//

#include "particles.h"

#include "common/angle_utils.h"

using namespace std;

//next free id for our particles
UniqueId firstFreeParticleId(const Particles& particles) {
    UniqueId next = 0;

    for (const ParticleInfo& p : particles) {
        if (p.id >= next) next = p.id + 1;
    }
    return next;
}

//splits a particle into 4 children and shares its data between them
vector<ParticleInfo> splitParticleWithTimes(const ParticleInfo& parent, const UniqueId startingId) {
    vector<ParticleInfo> kids;

    const double newRadius = (2 * parent.radius / (1 + sqrt(2))) / 2;
    //creation of the 4 children
    for (size_t i = 0; i < 4; ++i) {
        ParticleInfo child;
        child.id = startingId + static_cast<UniqueId>(i);
        child.radius = newRadius;
        //computes the center of each particle
        switch (i) {
        case 0: child.position.x = parent.position.x - newRadius;
            child.position.y = parent.position.y - newRadius;
            break;
        case 1: child.position.x = parent.position.x - newRadius;
            child.position.y = parent.position.y + newRadius;
            break;
        case 2: child.position.x = parent.position.x + newRadius;
            child.position.y = parent.position.y - newRadius;
            break;
        case 3: child.position.x = parent.position.x + newRadius;
            child.position.y = parent.position.y + newRadius;
            break;
        }

        //shares the explosion times between the particles
        for (size_t level = 1; level < parent.explosionTimes.size(); ++level) {
            size_t size_part = parent.explosionTimes[level].size() / 4; //size of a share, 4 -> 4 children
            size_t debut = i * size_part;
            size_t fin = debut + size_part;

            vector<double> timeExploding_childs;
            for (size_t j = debut; j < fin; ++j) {
                timeExploding_childs.push_back(parent.explosionTimes[level][j]);
            }
            child.explosionTimes.push_back(timeExploding_childs);
        }
        kids.push_back(child);
    }
    return kids;
}

//written lower : tells if the robot really pushes into the particle
static bool particleDoesNotApproach(const RobotInfo& robot, const ParticleInfo& particle, Time dt);

/**
 * @brief finds the next contact between a robot and a particle
 *
 * @param robots : list of the robots
 * @param particles : list of the particles
 * @param limit : max search time
 * @param wantCapture : true to look for a capture, false to look for a block
 * @param outTime : (output) time of the contact
 * @param outRobot : (output) ID of the robot that touches
 * @return True if such a contact happens before the limit
 */
static bool nextParticleContact(const Robots& robots, const Particles& particles, const Time limit,
                                const bool wantCapture, Time& outTime, int& outRobot) {
    bool found = false;

    for (const RobotInfo& robot : robots) {
        for (const ParticleInfo& particle : particles) {
            Time contactTime = 0.0;

            //if the robot does not touch the particle inside dt -> continue
            if (!findParticleContact(robot, particle, limit, contactTime)) continue;
            //if we want to decontaminate but we cannot -> continue
            if (canDecontaminate(getRobotAfterTime(robot, contactTime), particle) != wantCapture)
                continue;

            //Contact already there at t=0 : it counts as a block only if the robot
            //really pushes into it. Otherwise a robot turning on the spot would
            //hide the real coming contacts of the other robots.
            if (!wantCapture && contactTime <= 0.0
                && particleDoesNotApproach(robot, particle, limit))
                continue;

            if (!found || contactTime < outTime) {
                outTime = contactTime;
                outRobot = robot.id;
                found = true;
            }
        }
    }
    return found;
}

//predicts the time of the next decontamination
bool nextDecontaminationTime(const Robots& robots, const Particles& particles, Time limit, Time& outTime) {
    int robotId = -1;

    return nextParticleContact(robots, particles, limit, true, outTime, robotId);
}

//predicts the next "collision" against a particle
bool nextParticleBlock(const Robots& robots, const Particles& particles, const Time limit,
                       Time& outTime, int& outRobot) {
    return nextParticleContact(robots, particles, limit, false, outTime, outRobot);
}

// turns the robots stuck against a particle so they can leave it
void aimBlockedRobots(State& state, const Constraint& constraint) {
    //rotation speed
    const Speed maxPivot = std::min(constraint.maxForwardSpeed, constraint.maxBackwardSpeed);
    double dx = 0.0;
    double dy = 0.0;
    double diff = 0.0;
    double remaining = 0.0;
    double neededSpeed = 0.0;
    Speed pivot = 0.0;

    for (RobotInfo& robot : state.robots) {
        for (const ParticleInfo& particle : state.particles) {
            if (!isTouching(robot, particle)) continue;
            //if we touch it and we can eat it -> the particle is destroyed
            if (canDecontaminate(robot, particle)) continue;

            //angle between the blocking element and the robot
            dx = particle.position.x - robot.position.x;
            dy = particle.position.y - robot.position.y;
            //where the mouth of the robot is and where the particle is
            diff = normalizeDeg(std::atan2(dy, dx) * RAD2DEG - robot.angle);

            // we aim inside the capture angle
            remaining = std::abs(diff) - robot.captureAngle + ANGLE_TOLERANCE_DEG;
            if (remaining <= 0.0) {
                robot.leftSpeed = 0.0;
                robot.rightSpeed = 0.0;
                break;
            }

            // wheel speed that gives exactly 'remaining' in one interval
            neededSpeed = robot.radius * (remaining * DEG2RAD) / constraint.CommandTimeInterval;
            pivot = std::min(maxPivot, neededSpeed);

            //turn right or left, whichever avoids the obstacle the easiest
            robot.leftSpeed = (diff > 0.0) ? -pivot : pivot;
            robot.rightSpeed = (diff > 0.0) ? pivot : -pivot;
            break;
        }
    }
}

/**
 * @brief tells if the robot does NOT push into the particle during dt
 *
 * Turning on the spot keeps the gap the same : so it is not a block.
 *
 * @param robot : robot we check
 * @param particle : particle it is touching
 * @param dt : lookahead time
 * @return True if the gap never gets smaller, so the robot is not stuck
 */
static bool particleDoesNotApproach(const RobotInfo& robot, const ParticleInfo& particle,
                                    const Time dt) {
    double previousGap = particleGap(robot, particle);
    size_t sample = 0;

    for (sample = 1; sample <= PARTICLE_GAP_SAMPLES; ++sample) {
        const Time t = (dt * sample) / PARTICLE_GAP_SAMPLES;
        const double gap = particleGap(getRobotAfterTime(robot, t), particle);

        if (gap < previousGap - EPS) return false; //it pushes into it
        previousGap = gap;
    }
    return true;
}

bool isBlockedByParticle(const Particles& particles, const RobotInfo& robot, const Time dt) {
    for (const ParticleInfo& particle : particles) {
        if (!isTouching(robot, particle)) continue;
        //a cleanable particle does not stop the robot : it goes through while sucking it
        if (canDecontaminate(robot, particle)) continue;

        if (!particleDoesNotApproach(robot, particle, dt)) return true;
    }
    return false;
}

bool stopRobotsOnParticles(State& state) {
    bool stopped = false;

    for (const RobotInfo& robot : state.robots) {
        for (const ParticleInfo& particle : state.particles) {
            if (!isTouching(robot, particle)) continue;
            if (canDecontaminate(robot, particle)) continue;

            stopRobots(state.robots, robot.id, -1);
            stopped = true;
            break;
        }
    }
    return stopped;
}

//removes the cleaned particles and raises the score
bool decontaminateAtContact(State& state) {
    bool cleared = false;

    for (RobotInfo& robot : state.robots) {
        for (auto it = state.particles.begin(); it != state.particles.end();) {
            if (canDecontaminate(robot, *it)) {
                robot.score += M_PI * it->radius * it->radius; // area = score won
                it = state.particles.erase(it); //removes the particle
                cleared = true;
            }
            else {
                ++it;
            }
        }
    }
    return cleared;
}
