#include "mode.h"
#include "Plane.h"

void ModeStabilize::update()
{
}

void ModeStabilize::run()
{
    plane.stabilize_roll();
    plane.stabilize_pitch();
    stabilize_stick_mixing_direct();
    plane.stabilize_yaw();
}
