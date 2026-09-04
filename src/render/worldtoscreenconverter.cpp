#include "worldtoscreenconverter.h"

#include <algorithm>
#include <cmath>

// Keeps QRect's internal corner arithmetic well inside int. INT_MAX would not:
// it overflows on the very first addition.
constexpr double SCREEN_LIMIT = 1e6;

int toInt(double d) {
    if (!std::isfinite(d)) return 0;
    return int(std::round(std::clamp(d, -SCREEN_LIMIT, SCREEN_LIMIT)));
}

WorldToScreenConverter::WorldToScreenConverter(State const& state, QRect screen) noexcept
    : state(state), screen(screen), scale(1.)
{
    // Dividing by an empty world would make every coordinate below NaN.
    if (!state.hasValidWorld())
        return;

    double worldWidth = state.worldEnd.x - state.worldOrigin.x;
    double worldHeight = state.worldEnd.y - state.worldOrigin.y;
    scale = std::min(screen.width() / worldWidth, screen.height()/worldHeight);
}

double WorldToScreenConverter::x(double x) const noexcept {
    return ( x - state.worldOrigin.x ) * scale;
}

double WorldToScreenConverter::y(double y) const noexcept {
    return ( state.worldEnd.y - y ) * scale; // flip y upside down so that (0,0) is bottom left
}

double WorldToScreenConverter::r(double r) const noexcept {
    return r * scale;
}

QRect WorldToScreenConverter::circle(Position pos, double radius) const noexcept {
    double R = r(radius);
    return QRect(toInt(x(pos.x)-R),toInt(y(pos.y)-R),toInt(2*R),toInt(2*R));
}

QRect WorldToScreenConverter::world() const noexcept {
    return QRect(toInt(x(state.worldOrigin.x)),
                 toInt(y(state.worldEnd.y)),
                 toInt(r(state.worldEnd.x - state.worldOrigin.x)),
                 toInt(r(state.worldEnd.y - state.worldOrigin.y)));
}
