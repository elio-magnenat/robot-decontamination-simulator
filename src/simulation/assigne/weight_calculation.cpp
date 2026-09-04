//
// Created by Kilian Stockinger on 28.08.2026.
//
#include "weight_calculation.h"


#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <vector>

#include "common/angle_utils.h"
#include "common/Constants.h"

namespace {
    constexpr double INF = std::numeric_limits<double>::infinity();
}

// ---------------------------------------------------------------------------
// Particle helpers
// ---------------------------------------------------------------------------

// Particle area = what a capture is worth (doc section 30).
double particleArea(double radius) {
    return M_PI * radius * radius;
}

// Next explosion across all layers. INF if the particle never explodes.
Time nextExplosionTime(const ParticleInfo& particle, Time after) {
    Time soonest = INF;

    for (const auto& times : particle.explosionTimes) {
        for (const Time t : times) {
            if (t >= after && t < soonest) {
                soonest = t;
            }
        }
    }
    return soonest;
}

// Explosions already passed = layers lost.
std::size_t explosionsBefore(const ParticleInfo& particle, Time time) {
    std::size_t count = 0;

    for (const auto& times : particle.explosionTimes) {
        for (const Time t : times) {
            if (t < time) {
                ++count;
            }
        }
    }
    return count;
}

// Radius at a given time. Each past explosion removes one layer, and the
// radius never goes below zero.
double particleRadiusAt(const ParticleInfo& particle, Time time) {
    const std::size_t lost = explosionsBefore(particle, time);
    const std::size_t layers = std::max<std::size_t>(particle.explosionTimes.size(), 1);

    if (lost >= layers) {
        return 0.;
    }

    const double remaining = static_cast<double>(layers - lost)
                           / static_cast<double>(layers);

    return particle.radius * remaining;
}

// ---------------------------------------------------------------------------
// Grouping of neighbouring particles
// ---------------------------------------------------------------------------

double distanceBetween(const ParticleInfo& a, const ParticleInfo& b) {
    const double dx = a.position.x - b.position.x;
    const double dy = a.position.y - b.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Same test as for collisions: d <= rA + rB, plus a margin to catch particles
// that sit right next to each other without touching.
bool particlesAreNeighbours(const ParticleInfo& a, const ParticleInfo& b, Time time) {
    const double reach = particleRadiusAt(a, time)
                       + particleRadiusAt(b, time)
                       + CLUSTER_MARGIN;

    return distanceBetween(a, b) <= reach;
}

// A group of neighbouring particles. A robot going there may decontaminate
// several of them, so the total area is what counts.
struct ParticleCluster {
    std::vector<UniqueId> members;
    double totalArea = 0.;
};

// Cluster area indexed by particle id. Every member gets the same value:
// aiming at any of them gives access to the whole group.
std::map<UniqueId, double> buildClusterAreas(const std::vector<ParticleInfo>& particles,
                                             Time time) {
    const std::size_t n = particles.size();

    std::vector<int> clusterOf(n, -1);
    std::vector<ParticleCluster> clusters;

    // Step by step propagation: A next to B, B next to C, so A, B and C end up
    // in the same group even if A and C are far apart.
    for (std::size_t i = 0; i < n; ++i) {
        if (clusterOf[i] != -1) {
            continue;
        }

        const int id = static_cast<int>(clusters.size());
        clusters.emplace_back();

        std::vector<std::size_t> pile{i};
        clusterOf[i] = id;

        while (!pile.empty()) {
            const std::size_t current = pile.back();
            pile.pop_back();

            clusters[id].members.push_back(particles[current].id);
            clusters[id].totalArea += particleArea(particleRadiusAt(particles[current], time));

            for (std::size_t j = 0; j < n; ++j) {
                if (clusterOf[j] != -1) {
                    continue;
                }
                if (particlesAreNeighbours(particles[current], particles[j], time)) {
                    clusterOf[j] = id;
                    pile.push_back(j);
                }
            }
        }
    }

    std::map<UniqueId, double> areaById;

    for (const ParticleCluster& cluster : clusters) {
        for (const UniqueId id : cluster.members) {
            areaById[id] = cluster.totalArea;
        }
    }

    return areaById;
}

// ---------------------------------------------------------------------------
// PHASE 1 - quick estimate on every pair
// ---------------------------------------------------------------------------

void createRobotsRoutes(std::vector<RobotToParticle>& robotsRoutes,
                        const State& currentState,
                        const Constraint& constraint) {
    robotsRoutes.clear();
    robotsRoutes.reserve(currentState.robots.size() * currentState.particles.size());

    for (const RobotInfo& robot : currentState.robots) {
        for (const ParticleInfo& particle : currentState.particles) {
            robotsRoutes.push_back({
                robot,
                particle,
                estimateRemainingTime(robot, particle, constraint),
                0.
            });
        }
    }
}

// ---------------------------------------------------------------------------
// PHASE 2 + 3 - filter, then exact prediction
// ---------------------------------------------------------------------------

// Keeps the best pairs of each robot and replaces their estimated togo by the
// real ETA. After this all togo values are comparable.
void refineWithPrediction(std::vector<RobotToParticle>& robotsRoutes,
                          const State& currentState,
                          const Constraint& constraint,
                          const WorldBounds& world) {
    std::map<UniqueId, std::vector<std::size_t>> byRobot;

    for (std::size_t i = 0; i < robotsRoutes.size(); ++i) {
        byRobot[robotsRoutes[i].robotInfo.id].push_back(i);
    }

    std::vector<RobotToParticle> refined;
    refined.reserve(byRobot.size() * PREDICT_TOP_N);

    for (auto& [robotId, indices] : byRobot) {
        std::sort(indices.begin(), indices.end(),
                  [&](std::size_t a, std::size_t b) {
                      if (robotsRoutes[a].togo != robotsRoutes[b].togo) {
                          return robotsRoutes[a].togo < robotsRoutes[b].togo;
                      }
                      return a < b; // deterministic on ties
                  });

        const std::size_t count = std::min(indices.size(), PREDICT_TOP_N);

        for (std::size_t k = 0; k < count; ++k) {
            RobotToParticle route = robotsRoutes[indices[k]];

            const PredictedTrajectory prediction = predictTrajectory(
                route.robotInfo,
                route.particleInfo,
                constraint,
                world,
                currentState.time);

            // No capture means the pair is unusable.
            route.togo = prediction.captured ? prediction.eta : INF;

            refined.push_back(route);
        }
    }

    robotsRoutes = std::move(refined);
}

// Recomputes a single togo with the real physics, so an estimated value is
// never mixed with predicted ones.
double predictedTimeToGo(const RobotInfo& robot,
                         const ParticleInfo& particle,
                         const State& currentState,
                         const Constraint& constraint,
                         const WorldBounds& world) {
    const PredictedTrajectory prediction =
        predictTrajectory(robot, particle, constraint, world, currentState.time);

    return prediction.captured ? prediction.eta : INF;
}

// ---------------------------------------------------------------------------
// Weights
// ---------------------------------------------------------------------------

TemporaryWeight calculateRobotParticuleWeight(const RobotToParticle& route,
                                              int robotCount,
                                              const State& currentState,
                                              const std::map<UniqueId, double>& clusterAreas) {
    const Time explosion = nextExplosionTime(route.particleInfo, currentState.time);

    // Area of the group the particle belongs to, not just its own. Falls back
    // to the single area if the cluster was not computed.
    const Time arrival = currentState.time + route.togo;
    const auto it = clusterAreas.find(route.particleInfo.id);

    const double area = (it != clusterAreas.end())
                            ? it->second
                            : particleArea(particleRadiusAt(route.particleInfo, arrival));

    return {
        (explosion - currentState.time) - route.togo, // margin before explosion
        route.togo,
        area,
        robotCount
    };
}

double convertRobotParticuleWeight(const TemporaryWeight& weight) {
    // A pair that cannot capture must never win.
    if (!std::isfinite(weight.timeTra)) {
        return -INF;
    }

    // A particle that never explodes has an infinite margin, so it is zeroed
    // out and the rest of the score decides.
    const double margin = std::isfinite(weight.timeExpTra) ? weight.timeExpTra : 0.;

    // assignedRobots is still summed here, but coefRobots is 0 on purpose.
    // The number of robots already on a particle used to weigh in at this
    // point. Tests showed a weight term is the wrong tool for it: it shifted
    // every pair a little instead of dealing with the one robot that had no
    // target. Doing it in a separate pass, once we know which particles are
    // taken, works better. That is what assignLeftovers() does, together with
    // the one-robot-per-particle rule of matchOnePerParticle(). The term is
    // kept so the behaviour can be turned back on from Constants.h.
    return margin * coefExpTra
         + weight.timeTra * coefTra
         + weight.sizeParticule * coefParticule
         + weight.assignedRobots * coefRobots;
}

// ---------------------------------------------------------------------------
// Selection
// ---------------------------------------------------------------------------

// Every pair gets its weight. No particle is shared, so the number of robots
// already assigned is always zero here.
void scoreRobotParticule(std::vector<RobotToParticle>& robotsRoutes,
                         const State& currentState,
                         const std::map<UniqueId, double>& clusterAreas) {
    for (RobotToParticle& route : robotsRoutes) {
        route.weight = convertRobotParticuleWeight(
            calculateRobotParticuleWeight(route, 0, currentState, clusterAreas));
    }
}

// Greedy: best available pair first, one robot per particle and one particle
// per robot. Two robots aiming at the same particle are separated by their
// weight, the better one takes it and the other gets nothing and is stopped.
// Robots left with no free particle also come out without a target.
void matchOnePerParticle(std::vector<RobotToParticle>& robotsRoutes,
                         std::vector<RobotToParticle>& bestRobotsRoutes,
                         std::size_t robotCount) {
    std::vector<std::size_t> ordre(robotsRoutes.size());
    std::iota(ordre.begin(), ordre.end(), 0);

    std::sort(ordre.begin(), ordre.end(), [&](std::size_t a, std::size_t b) {
        if (robotsRoutes[a].weight != robotsRoutes[b].weight) {
            return robotsRoutes[a].weight > robotsRoutes[b].weight;
        }
        // On equal weights: for a given particle the weight only depends on it
        // (margin before explosion), so the ETA picks the better robot. Without
        // this the winner would just be the smallest id.
        if (robotsRoutes[a].togo != robotsRoutes[b].togo) {
            return robotsRoutes[a].togo < robotsRoutes[b].togo;
        }
        return a < b; // deterministic on ties
    });

    std::set<UniqueId> robotsPris;
    std::set<UniqueId> particulesPrises;

    bestRobotsRoutes.clear();
    bestRobotsRoutes.reserve(robotCount);

    for (const std::size_t i : ordre) {
        const RobotToParticle& route = robotsRoutes[i];

        if (!std::isfinite(route.togo)) {
            continue; // capture impossible
        }
        if (robotsPris.count(route.robotInfo.id) > 0) {
            continue;
        }
        if (particulesPrises.count(route.particleInfo.id) > 0) {
            continue;
        }

        bestRobotsRoutes.push_back(route);
        robotsPris.insert(route.robotInfo.id);
        particulesPrises.insert(route.particleInfo.id);

        if (robotsPris.size() == robotCount) {
            break;
        }
    }
}

// Catch-up: a robot can end up without a target while free particles remain,
// because all of its PREDICT_TOP_N candidates were taken. It then gets the
// closest free particle.
void assignLeftovers(std::vector<RobotToParticle>& bestRobotsRoutes,
                     const State& currentState,
                     const Constraint& constraint,
                     const WorldBounds& world,
                     bool analyticAssignmentOnly,
                     const std::map<UniqueId, double>& clusterAreas) {
    std::set<UniqueId> robotsPris;
    std::set<UniqueId> particulesPrises;

    for (const RobotToParticle& route : bestRobotsRoutes) {
        robotsPris.insert(route.robotInfo.id);
        particulesPrises.insert(route.particleInfo.id);
    }

    for (const RobotInfo& robot : currentState.robots) {
        if (robotsPris.count(robot.id) > 0) {
            continue;
        }
        if (particulesPrises.size() == currentState.particles.size()) {
            break; // nothing free left, remaining robots will be stopped
        }

        // Free particles, from the closest to the furthest.
        std::vector<const ParticleInfo*> candidates;

        for (const ParticleInfo& particle : currentState.particles) {
            if (particulesPrises.count(particle.id) == 0) {
                candidates.push_back(&particle);
            }
        }

        std::sort(candidates.begin(), candidates.end(),
                  [&](const ParticleInfo* a, const ParticleInfo* b) {
                      const Time ta = estimateRemainingTime(robot, *a, constraint);
                      const Time tb = estimateRemainingTime(robot, *b, constraint);
                      if (ta != tb) return ta < tb;
                      return a->id < b->id; // deterministic on ties
                  });

        for (const ParticleInfo* particle : candidates) {
            const Time togo = analyticAssignmentOnly
                      ? estimateRemainingTime(robot, *particle, constraint)
                      : predictedTimeToGo(robot, *particle, currentState,
                                          constraint, world);
            if (!std::isfinite(togo)) {
                continue; // capture impossible
            }

            RobotToParticle route{robot, *particle, togo, 0.};
            route.weight = convertRobotParticuleWeight(
                calculateRobotParticuleWeight(route, 0, currentState, clusterAreas));

            bestRobotsRoutes.push_back(route);
            robotsPris.insert(robot.id);
            particulesPrises.insert(particle->id);
            break;
        }
    }
}

void stopUnassignedRobots(Robots& robots, const std::vector<Trajectory>& assignments) {
    std::set<UniqueId> assigned;

    for (const Trajectory& trajectory : assignments) {
        assigned.insert(trajectory.robot.id);
    }

    for (RobotInfo& robot : robots) {
        if (assigned.count(robot.id) == 0) {
            robot.leftSpeed = 0.;
            robot.rightSpeed = 0.;
        }
    }
}

// Would another robot do better on this particle? The weight is the same for
// everyone on a given particle, so the ETA decides. Robots already locked and
// those with their own target are skipped. On a tie the robot in place keeps
// the particle.
bool hasBetterCandidate(const State& currentState,
                        const Constraint& constraint,
                        const std::map<UniqueId, UniqueId>& previousAssignment,
                        const std::set<UniqueId>& lockedRobots,
                        const RobotInfo& robot,
                        const ParticleInfo& particle) {
    const Time mine = estimateRemainingTime(robot, particle, constraint);

    for (const RobotInfo& other : currentState.robots) {
        if (other.id == robot.id) continue;
        if (lockedRobots.count(other.id) > 0) continue; // already gone elsewhere

        const auto prevIt = previousAssignment.find(other.id);
        if (prevIt != previousAssignment.end() && prevIt->second != particle.id) {
            continue; // busy with its own target
        }

        if (estimateRemainingTime(other, particle, constraint) < mine - EPS) {
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// Global assignment
// ---------------------------------------------------------------------------

std::vector<Trajectory> assignRobotParticule(const State& currentState,
                                             const Constraint& constraint,
                                             std::map<UniqueId, UniqueId>& previousAssignment) {
    const WorldBounds world{currentState.worldOrigin, currentState.worldEnd};
    std::vector<Trajectory> robotParticule;
    std::vector<RobotToParticle> robotsRoutes;
    std::vector<RobotToParticle> bestRobotsRoutes;

    if (currentState.robots.empty() || currentState.particles.empty()) {
        previousAssignment.clear();
        return robotParticule;
    }

    // --- Sticky assignment: lock the robots already on their way ------------
    std::set<UniqueId> lockedRobots;
    std::set<UniqueId> lockedParticles;

    for (const RobotInfo& robot : currentState.robots) {
        const auto prevIt = previousAssignment.find(robot.id);
        if (prevIt == previousAssignment.end()) continue;

        // Does the target particle still exist?
        const ParticleInfo* target = nullptr;
        for (const ParticleInfo& p : currentState.particles) {
            if (p.id == prevIt->second) { target = &p; break; }
        }
        if (!target) continue;

        // A particle is allowed only one robot.
        if (lockedParticles.count(target->id) > 0) continue;

        // Is the robot heading towards its target? (angle < MAX_ARC_TURN_DEG)
        const double targetAngle = std::atan2(target->position.y - robot.position.y,
                                              target->position.x - robot.position.x) * RAD2DEG;
        const double diff = normalizeDeg(targetAngle - robot.angle);

        if (std::abs(diff) > MAX_ARC_TURN_DEG) continue;

        // The lock must not keep a worse robot on the particle: if another free
        // robot does better, the pair is released and both are compared in the
        // greedy pass.
        if (hasBetterCandidate(currentState, constraint, previousAssignment,
                               lockedRobots, robot, *target)) {
            continue;
        }

        // Lock this pair
        robotParticule.push_back({robot, *target});
        lockedRobots.insert(robot.id);
        lockedParticles.insert(target->id);
    }

    // If everything is locked, we are done
    if (lockedRobots.size() == currentState.robots.size() ||
        lockedParticles.size() == currentState.particles.size()) {
        // Update previousAssignment with the final result
        previousAssignment.clear();
        for (const Trajectory& t : robotParticule) {
            previousAssignment[t.robot.id] = t.particle.id;
        }
        return robotParticule;
    }

    // Build a filtered state without the locked robots and particles
    State freeState = currentState;
    freeState.robots.erase(
        std::remove_if(freeState.robots.begin(), freeState.robots.end(),
                       [&](const RobotInfo& r) { return lockedRobots.count(r.id) > 0; }),
        freeState.robots.end());
    freeState.particles.erase(
        std::remove_if(freeState.particles.begin(), freeState.particles.end(),
                       [&](const ParticleInfo& p) { return lockedParticles.count(p.id) > 0; }),
        freeState.particles.end());

    // --- Normal algorithm on the free robots and particles -------------------

    // Groups are computed once for the whole pass.
    const std::map<UniqueId, double> clusterAreas =
        buildClusterAreas(freeState.particles, freeState.time);

    // With very expensive constraints, skip the full predictions during the
    // robot-particle assignment.
    const bool analyticAssignmentOnly =
        constraint.CommandTimeInterval < ANALYTIC_ASSIGNMENT_INTERVAL_THRESHOLD ||
        (constraint.maxBackwardSpeed > constraint.maxForwardSpeed &&
         freeState.robots.size() * freeState.particles.size()
             >= ANALYTIC_ASSIGNMENT_PAIR_THRESHOLD);

    createRobotsRoutes(robotsRoutes, freeState, constraint);

    // Keep the accurate prediction in normal scenarios.
    if (!analyticAssignmentOnly) {
        refineWithPrediction(robotsRoutes, freeState, constraint, world);
    }

    scoreRobotParticule(robotsRoutes, freeState, clusterAreas);

    matchOnePerParticle(
        robotsRoutes,
        bestRobotsRoutes,
        freeState.robots.size()
    );

    assignLeftovers(
        bestRobotsRoutes,
        freeState,
        constraint,
        world,
        analyticAssignmentOnly,
        clusterAreas
    );
    // Merge the free results with the locked pairs
    robotParticule.reserve(robotParticule.size() + bestRobotsRoutes.size());

    for (const RobotToParticle& route : bestRobotsRoutes) {
        robotParticule.push_back({route.robotInfo, route.particleInfo});
    }

    // Update previousAssignment with the final result
    previousAssignment.clear();
    for (const Trajectory& t : robotParticule) {
        previousAssignment[t.robot.id] = t.particle.id;
    }

    return robotParticule;
}
