#include "Blimp.h"
/*
 * Init and run calls for stabilize flight mode
 */

// accel_run - runs the main manual controller
// should be called at 100hz or more
void ModeAccel::run()
{
    float desired_accel_x = channel_front->get_control_in() / float(RC_SCALE) * g.max_xy_accel;
    float desired_accel_y = channel_right->get_control_in() / float(RC_SCALE) * g.max_xy_accel;

    Vector3f accel_bf = blimp.ahrs.get_accel();
    Vector3f target_accel = Vector3f(desired_accel_x, desired_accel_y, 0);

    Vector3f accel_error = target_accel - accel_bf;

    accel_xy_pid.set_input(accel_error);

    Vector2f actuator = accel_xy_pid.get_pid();

    motors->right_out = -actuator.y;
    motors->front_out = -actuator.x;

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(Fins::DesiredSpoolState::SHUT_DOWN);
    } else {
        motors->set_desired_spool_state(Fins::DesiredSpoolState::THROTTLE_UNLIMITED);
    }



}
