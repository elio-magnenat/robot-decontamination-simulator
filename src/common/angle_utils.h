#ifndef PIN_2026_ALPINE_ANGLE_UTILS_H
#define PIN_2026_ALPINE_ANGLE_UTILS_H
#include <cmath>


constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

//Ecart signe, dans [-180,180[ : sert a comparer un cap a captureAngle.
inline double normalizeDeg(double deg) {
    while (deg > 180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
}

//Cap absolu, dans [0,360[ : c'est le contrat de RobotInfo::angle dans le JSON.
inline double normalizeDeg360(double deg) {
    deg = std::fmod(deg, 360.0);
    if (deg < 0.0) deg += 360.0;
    //fmod d'un tres petit negatif peut arrondir pile sur 360 apres l'ajout
    if (deg >= 360.0) deg = 0.0;
    return deg;
}

#endif //PIN_2026_ALPINE_ANGLE_UTILS_H
