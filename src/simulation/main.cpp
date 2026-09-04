#include <iomanip>
#include <iostream>

#include "engine/command.h"
#include "world/event.h"
#include "world/explosion.h"
#include "engine/interval.h"
#include "io/io.h"
#include "world/particles.h"
#include "trajectory/trajectory_command.h"
#include "common/State.h"
#include "common/TimeLine.h"
#include <map>
#include  "assigne/weight_calculation.h"

using namespace std;

int main(const int argc, char* argv[]) {
    State state = {};
    TimeLine timeline;
    Time currentTime = 0.0;
    Time dt = 0.0;
    size_t iteration = 0;
    UniqueId nextParticleId = 0;
    bool collidedLastStep = false;
    bool usePlanner = true; //--no-traj : driving by the trajectory planner
    bool stateOk = false;
    bool constraintOk = false;
    Score totalScore = 0.0;
    std::string outputFile = "";

    std::map<UniqueId, UniqueId> previousAssignment;

    //How to use the program : the timeline name is not given, it is
    //built from the two input files (see buildTimelineName).
    if (argc < 3) {
        std::cerr << "[ERREUR] Usage: Setup <state.stat> <constraints.constraints> [--no-traj]"
            << std::endl;
        return ERROR::INVALID_FILE;
    }

    //only options are accepted after the two input files
    for (int i = 3; i < argc; ++i) {
        if (string(argv[i]) == "--no-traj") {
            usePlanner = false;
        }
        else {
            std::cerr << "[ERREUR] Option inconnue : " << argv[i] << std::endl;
            return ERROR::INVALID_FILE;
        }
    }

    // 1. load the State from argv[1]
    std::cout << "[INFO] Chargement du fichier d'état         : " << argv[1] << std::endl;
    state = OpenJSON_State(argv[1], &stateOk);
    if (!stateOk) {
        std::cerr << "[ERREUR] Impossible de lire l'état depuis " << argv[1] << std::endl;
        return ERROR::INVALID_FILE;
    }
    timeline.states.push_back(state); //put the initial state into the timeline

    // 2. load the constraints (CommandTimeInterval, maxSpeed...) from argv[2]
    std::cout << "[INFO] Chargement du fichier de contraintes : " << argv[2] << std::endl;
    const Constraint constraint = OpenJSON_Constraint(argv[2], &constraintOk);
    if (!constraintOk) {
        std::cerr << "[ERREUR] Impossible de lire les contraintes depuis " << argv[2] << std::endl;
        return ERROR::INVALID_FILE;
    }

    dt = constraint.CommandTimeInterval;
    //a zero interval would loop forever at t = 0
    if (dt <= 0.0) {
        std::cerr << "[ERREUR] CommandTimeInterval invalide (" << dt << ")" << std::endl;
        return ERROR::INVALID_FILE;
    }
    nextParticleId = firstFreeParticleId(state.particles);

    // 3. compute the TimeLine, one command interval at a time
    std::cout << "[INFO] Début de la simulation (dt = " << dt << "s)"
        << (usePlanner ? " [pilotage : planificateur de trajectoire]" : " [pilotage : direction]")
        << "..." << std::endl;

    // Warning about the execution time
    const auto start{std::chrono::steady_clock::now()};
    //time spent since 'start', to check again at each loop
    auto elapsedSeconds = [&start]() {
        return std::chrono::duration<double>{std::chrono::steady_clock::now() - start}.count();
    };

    double elapsed = 0.0;
    bool warningDisplay = false;

    while (!state.particles.empty() && elapsedSeconds() < SIMULATION_TIMEOUT_SECONDS) {
        //keep looping while the board is not empty
        iteration++;

        if (elapsedSeconds() > SIMULATION_WARNING_SECONDS && !warningDisplay) {
            std::cout <<
                "[WARNING] La simulation prend plus de 5 minutes. Veuillez vérifier les paramètres et les contraintes."
                << std::endl;
            warningDisplay = true;
        }

        std::cout << "\r[SIMU] Itération : " << iteration
            << " | Temps : " << std::fixed << std::setprecision(2) << currentTime << "s"
            << " | Particules restantes : " << state.particles.size()
            << std::flush;

        //driving : find the best target and compute the speeds to reach it
        const std::vector<Trajectory> assignments =
            assignRobotParticule(state, constraint, previousAssignment);

        // if usePlanner is true, the trajectory planner is used to compute the commands for each robot
        if (usePlanner) {
            //planner : search with lookahead, contact and capture found by dichotomy
            applyPlannedCommands(state, assignments, constraint);
        }
        else {
            applyCommands(state.robots, assignments, constraint);
        }
        //more robots than particles : the ones left without a target are stopped
        stopUnassignedRobots(state.robots, assignments);
        //collisions -> a blocked robot must turn on the spot
        unblockRobots(state.robots, constraint, dt);
        //stop the robot on a collision (wall or other robot)
        cancelBlockedCommands(state.robots, state.particles, state.worldOrigin, state.worldEnd, dt);
        //aim again the robots that were blocked before
        aimBlockedRobots(state, constraint);


        state.time = currentTime;
        if (collidedLastStep || isEvent(timeline.states.back(), state)) {
            timeline.states.push_back(state);
            //a new state goes in the timeline only on an important event or a collision
        }

        //move the simulation forward until the next time interval
        collidedLastStep = runInterval(state, timeline, currentTime, currentTime + dt,
                                       nextParticleId);

        //time interval x is done, go to the next one
        currentTime += dt;
        state.time = currentTime;

        //split the particles if it is time
        explodeDueParticles(state, currentTime, nextParticleId);
    }
    elapsed = elapsedSeconds();

    std::cout << "\n[INFO] Durée d'exécution de la simulation : "
        << std::fixed << std::setprecision(2) << elapsed << "s" << std::endl;

    if (elapsed >= SIMULATION_TIMEOUT_SECONDS) {
        std::cerr <<
            "[ERREUR] La simulation a dépassé la limite de temps de 10 minutes, veuilliez vérifier les paramètres et les contraintes."
            << std::endl;
        return ERROR::TIMEOUT;
    }

    timeline.states.push_back(state);

    std::cout << "[INFO] Simulation terminée avec succès en " << iteration << " itérations." << std::endl;

    //final score : sum of the areas cleaned by each robot
    for (const RobotInfo& robot : state.robots) {
        totalScore += robot.score;
        std::cout << "[INFO] Robot " << robot.id << " : score = "
            << std::fixed << std::setprecision(2) << robot.score << std::endl;
    }
    std::cout << "[INFO] Score total                   : "
        << std::fixed << std::setprecision(2) << totalScore << std::endl;

    // 4. write the result : <state>_<constraints>_<group>.tlin, next to the state
    outputFile = buildTimelineName(argv[1], argv[2]);
    std::cout << "[INFO] Exportation de la TimeLine vers      : " << outputFile << std::endl;
    if (exportTimeline(outputFile, timeline)) {
        cerr << "[ERREUR] Échec de l'exportation du fichier vers " << outputFile << std::endl;
        return ERROR::UNAVAILABLE_WRITING;
    }

    std::cout << "[INFO] Exportation terminée." << std::endl;
    return 0;
}
