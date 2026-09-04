#include "interval.h"

#include "world/collision.h"
#include "command.h"
#include "world/explosion.h"
#include "world/particles.h"
#include "common/Constants.h"

bool runInterval(State& state, TimeLine& timeline, const Time from, const Time to, UniqueId& nextParticleId) {
    //flags
    bool collided = false;
    bool cleared = false;
    bool exploded = false;
    bool hitNow = false;
    bool blockNow = false;
    bool wallNow = false;
    //times
    Time subTime = from;
    Time explosionAt = 0.0;
    Time contactAt = 0.0;
    Time collisionAt = 0.0;
    Time blockAt = 0.0;
    Time wallAt = 0.0;
    Time stopAt = 0.0;
    //id
    int hitA = -1;
    int hitB = -1;
    int blockedRobot = -1;
    int wallRobot = -1;

    while (subTime < to - EPS) {
        // while we have not reached the end of the time
        explosionAt = 0.0;
        contactAt = 0.0;
        collisionAt = 0.0;
        blockAt = 0.0;
        stopAt = to;
        wallAt = 0.0;
        hitNow = false;
        blockNow = false;
        wallNow = false;

        //explosion before the end ?
        if (nextExplosionTime(state.particles, subTime, to, explosionAt)) {
            stopAt = explosionAt;
        }
        //a particle cleaned ?
        if (nextDecontaminationTime(state.robots, state.particles,
                                    stopAt - subTime, contactAt)
            && subTime + contactAt < stopAt) {
            stopAt = subTime + contactAt;
        }

        //robot stuck in a particle ?
        if (nextParticleBlock(state.robots, state.particles, stopAt - subTime,
                              blockAt, blockedRobot)) {
            if (blockAt <= MIN_SUBSTEP) {
                for (RobotInfo& robot : state.robots) {
                    if (robot.id != blockedRobot) continue;
                    if (!isBlockedByParticle(state.particles, robot, stopAt - subTime)) continue;

                    stopRobots(state.robots, blockedRobot, -1);
                    collided = true;
                }
            }
            else if (subTime + blockAt < stopAt - MIN_SUBSTEP) {
                stopAt = subTime + blockAt;
                blockNow = true;
            }
        }

        //collision entre robots
        if (findFirstCollision(state.robots, stopAt - subTime, collisionAt, hitA, hitB)) {
            if (collisionAt <= MIN_SUBSTEP) {
                stopRobots(state.robots, hitA, hitB); //if stuck we stop the motors
                collided = true;
            }
            else if (subTime + collisionAt < stopAt - MIN_SUBSTEP) {
                //else we stop before the contact
                stopAt = subTime + collisionAt;
                hitNow = true;
            }
        }

        //collision with a wall ?
        if (nextWallContact(state.robots, state.worldOrigin, state.worldEnd, stopAt - subTime, wallAt, wallRobot)) {
            if (wallAt <= MIN_SUBSTEP) {
                stopRobots(state.robots, wallRobot, -1);
                collided = true;
            }
            else if (subTime + wallAt < stopAt - MIN_SUBSTEP) {
                stopAt = subTime + wallAt;
                wallNow = true;
            }
        }
        //we move everything until stopAt
        if (advanceWithCollisions(state.robots, state.worldOrigin, state.worldEnd, stopAt - subTime)) {
            collided = true;
        }
        subTime = stopAt;

        if (hitNow) {
            //on a crash we stop the robots involved
            stopRobots(state.robots, hitA, hitB);
            collided = true;
        }

        //cleaning of the particles
        cleared = decontaminateAtContact(state);

        //blocking particle = we stop the robot
        if (blockNow) stopRobots(state.robots, blockedRobot, -1);

        if (wallNow) {
            //wall = we stop the robot
            stopRobots(state.robots, wallRobot, -1);
            collided = true;
        }

        exploded = (subTime < to - EPS)
            && explodeDueParticles(state, subTime, nextParticleId);

        if (cleared || exploded || hitNow || blockNow || wallNow) {
            cancelBlockedCommands(state.robots, state.particles, state.worldOrigin, state.worldEnd,
                                  to - subTime);

            state.time = subTime;
            timeline.states.push_back(state);
        }
    }
    return collided;
}
