#include "arc_trajectory.h"
#include "common/Constants.h"

std::vector<WheelCommand> getArcCommands(const Constraint& constraint) {
    std::vector<WheelCommand> arcs;

    const Speed maxSpeed = constraint.maxForwardSpeed;

    if (maxSpeed <= 0.0) {
        return arcs;
    }
    //Generates left and right arc commands by reducing the speed of the inner wheel for each predefined ratio
    for (const double ratio : ARC_SPEED_RATIOS) {
        const Speed innerSpeed = maxSpeed * ratio;
        arcs.push_back({innerSpeed,maxSpeed});
        arcs.push_back({maxSpeed,innerSpeed});
    }

    return arcs;
}