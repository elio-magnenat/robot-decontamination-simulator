//
// Created by candas on 28.08.26.
//
// Verification rapide de la fonction d'evaluation.
//
// Les tests disent "passe" ou "echoue" ; ce programme montre POURQUOI une
// commande gagne, en affichant le detail des termes du score pour chaque
// candidat sur un scenario unique. C'est l'outil a utiliser pour regler les
// poids de ScoreWeights.
//
// Le banc de comparaison baseline/optimiseur n'a pas sa place ici : il
// deroule des simulations completes sur tout Data/Training et merite sa
// propre cible.

#include <cstdio>

#include "simulation/tests/test_helpers.h"

int main()
{
    const EvalContext ctx = makeContext();   // dt = 1, arriere 5, avant 20

    const RobotInfo robot = makeRobot(0.0, 0.0, 0.0);
    const ParticleInfo particle = makeParticle(60.0, 0.0);

    std::printf("cible devant, a %.0f px\n", gap(robot, particle));
    dump(scoreAll(robot, particle, ctx));
    return 0;
}
