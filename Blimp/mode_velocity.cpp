#include "Blimp.h"
/*
 * Init and run calls for stabilize flight mode
 */

// accel_run - runs the main manual controller
// should be called at 100hz or more
void ModeVelocity::run()
{
    float desired_vel_x = channel_front->get_control_in() / float(RC_SCALE) * g.max_xy_vel;
    float desired_vel_y = channel_right->get_control_in() / float(RC_SCALE) * g.max_xy_vel;

    Vector3f vel_ef = blimp.ahrs.get_velocity_NED();
    Vector3f vel_bf = blimp.ahrs.get_rotation_body_to_ned().transposed() * vel_ef;

    Vector3f target_vel = Vector3f(desired_vel_x, desired_vel_y, 0);

    Vector3f vel_error = target_vek - vel_bf;

    vel_xy_pid.set_input(vel_error);

    Vector2f target_accel = vel_xy_pid.get_pid();

    Vector3f accel_bf = blimp.ahrs.get_accel();
    Vector2f accel_bf_xy = Vector2f(constrain_float(accel_bf.x, -g.max_xy_accel, g.max_xy_accel),
                                    constrain_float(accel_bf.y, -g.max_xy_accel, g.max_xy_accel));

    Vector2f accel_error = target_accel - accel_bf_xy;

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
