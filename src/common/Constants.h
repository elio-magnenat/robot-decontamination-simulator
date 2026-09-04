#ifndef PIN_2026_ALPINE_CONSTANTS_H
#define PIN_2026_ALPINE_CONSTANTS_H

#include <cstddef>

// Tolerance generique pour comparer des vitesses et des distances (vg == vd,
// dt nul, egalite de deux commandes...). Volontairement plus fine que
// numeric_limits<float>::epsilon() : la relacher ferait passer deux vitesses
// distinctes pour egales dans getRobotAfterTime() et basculerait a tort sur le
// cas ligne droite.
// A NE PAS confondre avec la tolerance des dichotomies de collision, qui doit
// valoir numeric_limits<float>::epsilon() (regle metier) : voir collision.cpp
// et trajectory_collision.cpp.
static constexpr double EPS = 1e-9;

// UI
static constexpr short TICK = 24;
static constexpr int INTERVAL = 1000 / TICK;

// Qt
static constexpr double kSingleStateDuration = 99.99;
static constexpr int kSliderResolution = 1000;

// Identifiant du groupe, repris dans le nom des timelines produites :
// <state>_<constraints>_<nn>.tlin
static constexpr char GROUP_ID[] = "03";

// Simulation Run
static constexpr double SIMULATION_WARNING_SECONDS = 60.0 * 5;
static constexpr double SIMULATION_TIMEOUT_SECONDS = 60.0 * 10;

// Simulation
static constexpr double ANGLE_TOLERANCE_DEG = 5.0;
static constexpr double MAX_ARC_TURN_DEG = 90.0;
static constexpr double CONTACT_EPS = 1e-3;

// Simulation Collision
static constexpr double SAFETY_MARGIN = 1.25;
static constexpr double REPULSION_STRENGTH = 30.0;

// Coefficient pour le calcul des poids
static constexpr double coefExpTra = -0.12;
static constexpr double coefTra = -1.0;
static constexpr double coefParticule = 0.00175;
static constexpr double coefRobots = 0.0;

// Trajectory Planner
constexpr double FUTURE_DISCOUNT = 0.8907283495581166;
constexpr double TERMINAL_ETA_WEIGHT = 20.0;

constexpr double FAST_COMMAND_TERMINAL_ETA_WEIGHT = 25.75;
constexpr double FAST_COMMAND_INTERVAL_THRESHOLD = 0.30;

constexpr int LOOKAHEAD_DEPTH = 4;
constexpr std::size_t REFINE_TOP_COUNT = 3;
constexpr double REFINEMENT_RATIO = 0.05;

// Elagage : sous la racine on ne prolonge que les 3 meilleures commandes
constexpr std::size_t BRANCHING_TOP_COUNT = 3;

constexpr double SCORE_WEIGHT_DISTANCE = 0.6;
constexpr double SCORE_WEIGHT_ANGLE = 1.0;
constexpr double SCORE_WEIGHT_PROGRESS = 9.0;
constexpr double SCORE_WEIGHT_CAPTURE = 897.7428679862774;
constexpr double SCORE_WEIGHT_BLOCKED = 2.7328014555747133;
constexpr double SCORE_WEIGHT_OSCILLATION = 0.0;

constexpr double SEMI_TURN_DEG = 180.0;

// Trajectory Simulator : garde-fous de la boucle de simulation.
constexpr int MAX_STEPS = 10000;
constexpr int MAX_STAGNANT_STEPS = 5;

constexpr int CONTACT_SAMPLE_COUNT = 12;
constexpr int CONTACT_MINIMUM_ITERATIONS = 20;

constexpr int CAPTURE_SAMPLE_COUNT = 50;

constexpr int WORLD_SAMPLE_COUNT = 12;
constexpr int WORLD_MINIMUM_ITERATIONS = 20;

constexpr double CONTACT_PROBE_RATIO = 0.001;
constexpr double CONTACT_PROBE_EPSILON = 1e-7;

// Marge de contact : snapToContact laisse un residu d'arrondi positif.
constexpr double CONTACT_TOLERANCE = 1e-4;

// Trajectoires simples : rotation par paliers jusqu'au cap vise.
constexpr int MAX_ROTATION_STEPS = 10000;
constexpr double ANGLE_EPS = 1e-7;

// Arcs candidats : rapport vitesse roue interieure / roue exterieure.
constexpr double ARC_SPEED_RATIOS[] = {
    0.6638467467550947,
    0.5769097009316648,
    0.025166670603943227
};

// Nombre d'echantillons des balayages de prevision (murs, robots, particules).
constexpr int GAP_SAMPLES = 16;
constexpr int PARTICLE_GAP_SAMPLES = 16;

// Plus petit saut possible de la boucle d'intervalle : evite les boucles infinies.
constexpr double MIN_SUBSTEP = 1e-4;

constexpr std::size_t PREDICT_TOP_N = 3;

constexpr double CLUSTER_MARGIN = 40.0;

constexpr double ANALYTIC_ASSIGNMENT_INTERVAL_THRESHOLD = 0.5;
constexpr std::size_t ANALYTIC_ASSIGNMENT_PAIR_THRESHOLD = 80;


enum ERROR {
    SUCCESS = 0,
    EMPTY_NAME_FILE = 1,
    UNAVAILABLE_READING = 2,
    UNAVAILABLE_WRITING = 3,
    INVALID_FILE = 4,
    TIMEOUT = 5
};

#endif //PIN_2026_ALPINE_CONSTANTS_H
