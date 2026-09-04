//
// Created by candas on 28.08.26.
//

#ifndef PIN_2026_ALPINE_TEST_HELPERS_H
#define PIN_2026_ALPINE_TEST_HELPERS_H

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "common/Movement.h"
#include "../trajectory/trajectory_score.h"


// Robot minimal pour les scenarios de test.
inline RobotInfo makeRobot(double x, double y, Angle angle,
                           Distance radius = 10.0, Angle captureAngle = 30.0)
{
    RobotInfo robot;
    robot.id = 0;
    robot.position = {x, y};
    robot.radius = radius;
    robot.angle = angle;
    robot.captureAngle = captureAngle;
    robot.leftSpeed = 0.0;
    robot.rightSpeed = 0.0;
    robot.score = 0.0;
    return robot;
}

inline ParticleInfo makeParticle(double x, double y, Distance radius = 5.0)
{
    ParticleInfo particle;
    particle.id = 0;
    particle.position = {x, y};
    particle.radius = radius;
    return particle;
}

inline EvalContext makeContext(double dt = 1.0, double backward = 5.0, double forward = 20.0)
{
    EvalContext ctx;
    ctx.constraint = Constraint{dt, backward, forward};   // ordre reel du struct
    return ctx;
}

struct Candidate
{
    const char* name;
    WheelCommand command;
};

// Commandes candidates ecrites a la main, pour rester independant du generateur.
inline std::vector<Candidate> makeCandidates(const Constraint& constraint)
{
    const Speed forward = constraint.maxForwardSpeed;
    const Speed backward = constraint.maxBackwardSpeed;
    const Speed pivot = std::min(forward, backward);
    return {
        {"tout droit",   {forward,       forward}},
        {"arc gauche",   {forward * 0.5, forward}},
        {"arc droit",    {forward,       forward * 0.5}},
        {"rot. gauche",  {-pivot,        pivot}},
        {"rot. droite",  {pivot,         -pivot}},
        {"marche arr.",  {-backward,     -backward}},
    };
}

struct Scored
{
    std::string name;
    WheelCommand command;
    double score;
    ScoreBreakdown breakdown;
};

// Evalue tous les candidats, dans le meme ordre.
inline std::vector<Scored> scoreAll(const RobotInfo& robot, const ParticleInfo& particle,
                                    const EvalContext& ctx)
{
    std::vector<Scored> results;
    for (const Candidate& candidate : makeCandidates(ctx.constraint))
    {
        RobotInfo simulated = robot;
        simulated.leftSpeed = candidate.command.leftSpeed;
        simulated.rightSpeed = candidate.command.rightSpeed;
        const RobotInfo after =
            getRobotAfterTime(simulated, ctx.constraint.CommandTimeInterval);

        ScoreBreakdown breakdown;
        const double score = evaluateTrajectoryStep(robot, after, particle,
                                                    candidate.command, ctx, &breakdown);
        results.push_back({candidate.name, candidate.command, score, breakdown});
    }
    return results;
}

inline const Scored& best(const std::vector<Scored>& scored)
{
    const Scored* winner = &scored.front();
    for (const Scored& candidate : scored)
        if (candidate.score > winner->score) winner = &candidate;
    return *winner;
}

// Renvoie un pointeur : l'appelant decide quoi faire si le nom est absent.
inline const Scored* find(const std::vector<Scored>& scored, const std::string& name)
{
    for (const Scored& candidate : scored)
        if (candidate.name == name) return &candidate;
    return nullptr;
}

// Detail des termes : le framework dit "faux", pas quel terme a decide.
inline void dump(const std::vector<Scored>& scored)
{
    std::printf("    %-14s %8s %8s %8s %8s %10s\n",
                "commande", "dist", "angle", "progr", "oscil", "total");
    for (const Scored& s : scored)
        std::printf("    %-14s %8.3f %8.3f %8.3f %8.3f %10.3f\n",
                    s.name.c_str(), s.breakdown.distance, s.breakdown.angle,
                    s.breakdown.progress, s.breakdown.oscillation,
                    s.breakdown.total);
}

#endif //PIN_2026_ALPINE_TEST_HELPERS_H
