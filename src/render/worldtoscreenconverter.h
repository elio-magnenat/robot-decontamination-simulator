#ifndef WORLDTOSCREENCONVERTER_H
#define WORLDTOSCREENCONVERTER_H

#include "../common/State.h"
#include <QRect>

class WorldToScreenConverter
{
public:    
    WorldToScreenConverter(State const& state, QRect screen) noexcept;
    double x(double x) const noexcept;
    double y(double y) const noexcept;
    double r(double r) const noexcept;
    QRect circle(Position p, double r) const noexcept;
    QRect world() const noexcept;
private:
    State state;
    QRect screen;
    double scale;
};

#endif // WORLDTOSCREENCONVERTER_H
