#include "trajectory_timeline.h"

#include <algorithm>
#include <cmath>

//Replace a robot in the state with its updated information using his ID
static void replaceRobot(State& state,const RobotInfo& robot) {
    for (RobotInfo& stateRobot : state.robots) {
        if (stateRobot.id == robot.id) {
            stateRobot = robot;
            return;
        }
    }
}

//Remove the particle with the ID from the state
static void removeParticle(State& state,UniqueId particleId) {
    state.particles.erase(
        std::remove_if(
            state.particles.begin(),
            state.particles.end(),
            [particleId](const ParticleInfo& particle) {
                return particle.id == particleId;
            }
        ),
        state.particles.end()
    );
}


TimeLine trajectoryToTimeline(const State& initialState,const PredictedTrajectory& trajectory) {
    TimeLine timeline;

    //Create a simulation state for the beginning of each trajectory step with the wheel speeds that must be applied at that time
    for (const TrajectoryStep& step : trajectory.steps) {
        State state = initialState;

        state.time = step.startTime;

        RobotInfo robot = step.startRobot;
        robot.leftSpeed = step.command.leftSpeed;
        robot.rightSpeed = step.command.rightSpeed;

        replaceRobot(state,robot);

        timeline.states.push_back(state);
    }

    //Build the final state using the robot position at the end of the last trajectory step
    State finalState = initialState;
    if (!trajectory.steps.empty()) {
        const TrajectoryStep& lastStep = trajectory.steps.back();

        finalState.time = lastStep.startTime + lastStep.duration;

        RobotInfo finalRobot = lastStep.endRobot;

        replaceRobot(finalState,finalRobot);
    }
    else {
        finalState.time = initialState.time;
    }
    //If the target was captured, find the corresponding particle to update the robot score and remove it from the final state
    if (trajectory.captured) {
        const auto particleIt = std::find_if(finalState.particles.begin(),finalState.particles.end(),[&trajectory](const ParticleInfo& particle) {
            return particle.id == trajectory.particleId;
        });
        //Add the captured particle area to the score of the robot that captured it
        if (particleIt != finalState.particles.end()) {
            for (RobotInfo& robot : finalState.robots) {
                if (robot.id == trajectory.robotId) {
                    robot.score += std::acos(-1.0) * particleIt->radius * particleIt->radius;
                    break;
                }
            }
        }
        //Remove the captured particle from the final state
        removeParticle(finalState,trajectory.particleId);
    }

    timeline.states.push_back(finalState);

    return timeline;
}