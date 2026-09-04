//
// Created by candas on 28.08.26.
//

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "common/doctest.h"

#include <cmath>

#include "common/angle_utils.h"
#include "simulation/tests/test_helpers.h"
#include "../trajectory/trajectory_collision.h"

namespace
{
    // Candidat par son nom, ou interruption du test si absent.
    const Scored& require(const std::vector<Scored>& scored, const std::string& name)
    {
      const Scored* found = find(scored, name);
      REQUIRE(found != nullptr);
      return *found;
    }
}

TEST_CASE("cible devant : tout droit doit gagner")
{
    const EvalContext ctx = makeContext();
    const RobotInfo robot = makeRobot(0.0, 0.0, 0.0);
    const ParticleInfo particle = makeParticle(60.0, 0.0);

    const std::vector<Scored> scored = scoreAll(robot, particle, ctx);
    REQUIRE(!scored.empty());

    const bool ok = best(scored).name == std::string("tout droit");
    CHECK(ok);
    if (!ok) dump(scored);
}

TEST_CASE("cible de cote : les candidats du bon cote doivent gagner")
{
    const EvalContext ctx = makeContext();
    const RobotInfo robot = makeRobot(0.0, 0.0, 0.0);

    // Cible a gauche : +60 degres.
    const ParticleInfo left = makeParticle(60.0 * std::cos(60.0 * DEG2RAD),
                                         60.0 * std::sin(60.0 * DEG2RAD));
    const std::vector<Scored> scored = scoreAll(robot, left, ctx);

    const bool arcs = require(scored, "arc gauche").score > require(scored, "arc droit").score;
    const bool rots = require(scored, "rot. gauche").score > require(scored, "rot. droite").score;
    CHECK(arcs);
    CHECK(rots);
    if (!arcs || !rots) dump(scored);
}

TEST_CASE("symetrie gauche/droite exacte")
{
    const EvalContext ctx = makeContext();
    const RobotInfo robot = makeRobot(0.0, 0.0, 0.0);

    const ParticleInfo left = makeParticle(60.0 * std::cos(60.0 * DEG2RAD),
                                         60.0 * std::sin(60.0 * DEG2RAD));
    const ParticleInfo right = makeParticle(left.position.x, -left.position.y);

    const std::vector<Scored> leftScored = scoreAll(robot, left, ctx);
    const std::vector<Scored> rightScored = scoreAll(robot, right, ctx);

    // Un branchement asymetrique dans la geometrie se verrait ici.
    const double arcGap = std::abs(require(leftScored, "arc gauche").score
                               - require(rightScored, "arc droit").score);
    const double rotGap = std::abs(require(leftScored, "rot. gauche").score
                               - require(rightScored, "rot. droite").score);
    CHECK(arcGap < 1e-9);
    CHECK(rotGap < 1e-9);
    if (arcGap >= 1e-9 || rotGap >= 1e-9) { dump(leftScored); dump(rightScored); }
}

TEST_CASE("au contact hors du cone : la rotation sur place doit gagner")
{
    const EvalContext ctx = makeContext();

    // Au contact (10 + 5), oriente a 45 deg pour un captureAngle de 30.
    const RobotInfo robot = makeRobot(0.0, 0.0, 45.0);
    const ParticleInfo particle = makeParticle(15.0, 0.0);

    const std::vector<Scored> scored = scoreAll(robot, particle, ctx);
    const Scored& winner = best(scored);

    const bool rotates = winner.name == std::string("rot. droite");
    CHECK(rotates);
    if (!rotates) dump(scored);

    // La capture appartient au simulateur : c'est lui qui doit la reconnaitre.
    const WorldBounds world{{-1000.0, -1000.0}, {1000.0, 1000.0}};
    CHECK(simulateCandidate(robot, {5.0, -5.0}, particle, world, 1.0).result
          == CommandStepResult::Captured);

    // Decale de 1e-9 px : sans tolerance de contact, la rotation ne capture plus.
    const RobotInfo drifted = makeRobot(-1e-9, 0.0, 45.0);
    CHECK(simulateCandidate(drifted, {5.0, -5.0}, particle, world, 1.0).result
          == CommandStepResult::Captured);
}

TEST_CASE("scores deterministes")
{
    const EvalContext ctx = makeContext();
    const RobotInfo robot = makeRobot(0.0, 0.0, 12.5);
    const ParticleInfo particle = makeParticle(37.0, 21.0);

    const std::vector<Scored> reference = scoreAll(robot, particle, ctx);

    for (int i = 0; i < 100; ++i)
    {
        const std::vector<Scored> again = scoreAll(robot, particle, ctx);
        for (size_t k = 0; k < reference.size(); ++k)
        {
            // Egalite exacte voulue : on mesure le determinisme, pas une proximite.
            CHECK(again[k].score == reference[k].score);
        }
    }
}

TEST_CASE("cible derriere : ne pas foncer tout droit")
{
    const EvalContext ctx = makeContext();
    const RobotInfo robot = makeRobot(0.0, 0.0, 0.0);
    const ParticleInfo particle = makeParticle(-60.0, 0.0);

    const std::vector<Scored> scored = scoreAll(robot, particle, ctx);
    const Scored& winner = best(scored);

    // Erreur de 180 deg : reculer ou tourner, jamais avancer.
    const bool ok = winner.name == std::string("marche arr.")
                 || winner.name == std::string("rot. gauche")
                 || winner.name == std::string("rot. droite");

    // Et tout droit doit etre le pire : il s'eloigne au maximum.
    const double straight = require(scored, "tout droit").score;
    bool worst = true;
    for (const Scored& candidate : scored)
        if (candidate.score < straight) worst = false;

    CHECK(ok);
    CHECK(worst);
    if (!ok || !worst) dump(scored);
}

TEST_CASE("proche mais mal oriente : corriger l'angle avant d'entrer")
{
    const EvalContext ctx = makeContext();

    // gap = 2 px mais erreur de 90 deg : foncer perdrait un intervalle.
    const RobotInfo robot = makeRobot(0.0, 0.0, -90.0);
    const ParticleInfo particle = makeParticle(17.0, 0.0);

    const std::vector<Scored> scored = scoreAll(robot, particle, ctx);
    const bool ok = best(scored).name != std::string("tout droit");
    CHECK(ok);
    if (!ok) dump(scored);
}

TEST_CASE("contraintes 5_20_1_backward : la marche arriere doit gagner")
{
    // Reculer est quatre fois plus rapide : aucun biais vers l'avant ne doit rester.
    const EvalContext ctx = makeContext(1.0, 20.0, 5.0);
    const RobotInfo robot = makeRobot(0.0, 0.0, 0.0);
    const ParticleInfo particle = makeParticle(-60.0, 0.0);

    const std::vector<Scored> scored = scoreAll(robot, particle, ctx);
    const bool ok = best(scored).name == std::string("marche arr.");
    CHECK(ok);
    if (!ok) dump(scored);
}

TEST_CASE("invariance d'echelle : longueurs et vitesses mises a l'echelle")
{
    // Longueurs et vitesses x k, dt constant : les scores doivent etre identiques.
    const double k = 3.0;

    const std::vector<Scored> reference =
        scoreAll(makeRobot(0.0, 0.0, 0.0, 10.0, 30.0),
                 makeParticle(60.0, 0.0, 5.0),
                 makeContext(1.0, 5.0, 20.0));

    const std::vector<Scored> scaled =
        scoreAll(makeRobot(0.0, 0.0, 0.0, 10.0 * k, 30.0),
                 makeParticle(60.0 * k, 0.0, 5.0 * k),
                 makeContext(1.0, 5.0 * k, 20.0 * k));

    REQUIRE(reference.size() == scaled.size());
    bool identical = true;
    for (size_t i = 0; i < reference.size(); ++i)
    {
        if (scaled[i].score != reference[i].score) identical = false;
        CHECK(scaled[i].score == reference[i].score);
    }
    if (!identical) { dump(reference); dump(scaled); }
}
