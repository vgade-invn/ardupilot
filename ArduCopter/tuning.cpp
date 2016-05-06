// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
  tables of tuning sets
 */
const uint8_t AP_Tuning_Copter::tuning_set_q_rate_roll_pitch[] = { TUNING_RATE_ROLL_KPI, TUNING_RATE_ROLL_KD,
                                                                  TUNING_RATE_PITCH_KPI, TUNING_RATE_PITCH_KD};
const uint8_t AP_Tuning_Copter::tuning_set_q_rate_roll[] =       { TUNING_RATE_ROLL_KPI, TUNING_RATE_ROLL_KD };
const uint8_t AP_Tuning_Copter::tuning_set_q_rate_pitch[] =      { TUNING_RATE_PITCH_KPI, TUNING_RATE_PITCH_KD };

// macro to prevent getting the array length wrong
#define TUNING_ARRAY(v) ARRAY_SIZE(v), v

// list of tuning sets
const AP_Tuning_Copter::tuning_set AP_Tuning_Copter::tuning_sets[] = {
    { TUNING_SET_RATE_ROLL_PITCH, TUNING_ARRAY(tuning_set_q_rate_roll_pitch) },
    { TUNING_SET_RATE_ROLL,       TUNING_ARRAY(tuning_set_q_rate_roll) },
    { TUNING_SET_RATE_PITCH,      TUNING_ARRAY(tuning_set_q_rate_pitch) },
    { TUNING_SET_NONE, 0, nullptr }
};

/*
  table of tuning names
 */
const AP_Tuning_Copter::tuning_name AP_Tuning_Copter::tuning_names[] = {
    { TUNING_RATE_ROLL_KPI, "RateRollPI" },
    { TUNING_RATE_ROLL_KD,  "RateRollD" },
    { TUNING_RATE_PITCH_KPI,"RatePitchPI" },
    { TUNING_RATE_PITCH_KD, "RatePitchD" },
    { TUNING_NONE, nullptr }
};

/*
  get a pointer to an AP_Float for a parameter, or NULL on fail
 */
AP_Float *AP_Tuning_Copter::get_param_pointer(uint8_t parm)
{
    switch(parm) {

    case TUNING_RATE_ROLL_KPI:
        // use P for initial value when tuning PI
        return &copter.attitude_control.get_rate_roll_pid().kP();

    case TUNING_RATE_ROLL_KP:
        return &copter.attitude_control.get_rate_roll_pid().kP();

    case TUNING_RATE_ROLL_KI:
        return &copter.attitude_control.get_rate_roll_pid().kI();

    case TUNING_RATE_ROLL_KD:
        return &copter.attitude_control.get_rate_roll_pid().kD();

    case TUNING_RATE_PITCH_KPI:
        return &copter.attitude_control.get_rate_pitch_pid().kP();

    case TUNING_RATE_PITCH_KP:
        return &copter.attitude_control.get_rate_pitch_pid().kP();

    case TUNING_RATE_PITCH_KI:
        return &copter.attitude_control.get_rate_pitch_pid().kI();

    case TUNING_RATE_PITCH_KD:
        return &copter.attitude_control.get_rate_pitch_pid().kD();

    case TUNING_RATE_YAW_KPI:
        return &copter.attitude_control.get_rate_yaw_pid().kP();

    case TUNING_RATE_YAW_KP:
        return &copter.attitude_control.get_rate_yaw_pid().kP();

    case TUNING_RATE_YAW_KI:
        return &copter.attitude_control.get_rate_yaw_pid().kI();

    case TUNING_RATE_YAW_KD:
        return &copter.attitude_control.get_rate_yaw_pid().kD();

    case TUNING_ANG_ROLL_KP:
        return &copter.attitude_control.get_angle_roll_p().kP();

    case TUNING_ANG_PITCH_KP:
        return &copter.attitude_control.get_angle_pitch_p().kP();

    case TUNING_ANG_YAW_KP:
        return &copter.attitude_control.get_angle_yaw_p().kP();

    case TUNING_PXY_P:
        return &copter.g.p_pos_xy.kP();

    case TUNING_PZ_P:
        return &copter.g.p_alt_hold.kP();

    case TUNING_VXY_P:
        return &copter.g.pi_vel_xy.kP();

    case TUNING_VXY_I:
        return &copter.g.pi_vel_xy.kI();

    case TUNING_VZ_P:
        return &copter.g.p_vel_z.kP();

    case TUNING_AZ_P:
        return &copter.g.pid_accel_z.kP();

    case TUNING_AZ_I:
        return &copter.g.pid_accel_z.kI();

    case TUNING_AZ_D:
        return &copter.g.pid_accel_z.kD();
        
    default:
        break;
    }
    return nullptr;
}


/*
  save a parameter
 */
void AP_Tuning_Copter::save_value(uint8_t parm)
{
    switch(parm) {
    // special handling of dual-parameters
    case TUNING_RATE_ROLL_KPI:
        save_value(TUNING_RATE_ROLL_KP);
        save_value(TUNING_RATE_ROLL_KI);
        break;
    case TUNING_RATE_PITCH_KPI:
        save_value(TUNING_RATE_PITCH_KP);
        save_value(TUNING_RATE_PITCH_KI);
        break;
    default:
        AP_Float *f = get_param_pointer(parm);
        if (f != nullptr) {
            f->save();
        }
        break;
    }
}

/*
  set a parameter
 */
void AP_Tuning_Copter::set_value(uint8_t parm, float value)
{
    switch(parm) {
    // special handling of dual-parameters
    case TUNING_RATE_ROLL_KPI:
        set_value(TUNING_RATE_ROLL_KP, value);
        set_value(TUNING_RATE_ROLL_KI, value);
        break;
    case TUNING_RATE_PITCH_KPI:
        set_value(TUNING_RATE_PITCH_KP, value);
        set_value(TUNING_RATE_PITCH_KI, value);
        break;
    default:
        AP_Float *f = get_param_pointer(parm);
        if (f != nullptr) {
            f->set_and_notify(value);
        }
        break;
    }
}
