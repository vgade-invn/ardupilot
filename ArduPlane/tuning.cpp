// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

const AP_Param::GroupInfo Tuning::var_info[] = {

    // @Param: CHAN
    // @DisplayName: Transmitter tuning channel
    // @Description: This sets the channel for transmitter tuning. This should be connected to a knob or slider on your transmitter. It needs to be setup to use the PWM range given by TUNE_CHAN_MIN to TUNE_CHAN_MAX
    // @Values: 0:Disable,1:Chan1,2:Chan3,3:Chan3,4:Chan4,5:Chan5,6:Chan6,7:Chan7,8:Chan8,9:Chan9,10:Chan10,11:Chan11,12:Chan12,13:Chan13,14:Chan14,15:Chan15,16:Chan16
    // @User: Standard
    AP_GROUPINFO_FLAGS("CHAN", 1, Tuning, channel, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: PARMSET
    // @DisplayName: Transmitter tuning parameter set
    // @Description: This sets which set of parameters will be tuned
    // @Values: 0:None,1:QuadRateRollPitch,2:QuadRateRoll,3:QuadRatePitch
    // @User: Standard
    AP_GROUPINFO("PARMSET", 2, Tuning, parmset, 0),

    // index 2 and 3 reserved for old MIN and MAX parameters
    
    // @Param: RANGE
    // @DisplayName: Transmitter tuning range
    // @Description: This sets the range over which tuning will change a parameter. A value of 2 means the tuning parameter will go from 0.5 times the start value to 2x the start value over the range of the tuning channel
    // @User: Standard
    AP_GROUPINFO("RANGE", 5, Tuning, range, 2.0f),

    // @Param: SELECTOR
    // @DisplayName: Transmitter tuning selector channel
    // @Description: This sets the channel for the transmitter tuning selector switch. This should be a 2 position switch, preferably spring loaded. A PWM above 1700 means high, below 1300 means low.
    // @Values: 0:Disable,1:Chan1,2:Chan3,3:Chan3,4:Chan4,5:Chan5,6:Chan6,7:Chan7,8:Chan8,9:Chan9,10:Chan10,11:Chan11,12:Chan12,13:Chan13,14:Chan14,15:Chan15,16:Chan16
    // @User: Standard
    AP_GROUPINFO("SELECTOR", 6, Tuning, selector, 0),

    // @Param: CHAN_MIN
    // @DisplayName: Transmitter tuning channel minimum pwm
    // @Description: This sets the PWM lower limit for the tuning channel
    // @Range: 900 2100
    // @User: Standard
    AP_GROUPINFO("CHAN_MIN", 7, Tuning, channel_min, 1000),

    // @Param: CHAN_MAX
    // @DisplayName: Transmitter tuning channel maximum pwm
    // @Description: This sets the PWM upper limit for the tuning channel
    // @Range: 900 2100
    // @User: Standard
    AP_GROUPINFO("CHAN_MAX", 8, Tuning, channel_max, 2000),
    
    AP_GROUPEND
};

/*
  tables of tuning sets
 */
static const Tuning::tuning_func tuning_set_q_rate_roll_pitch[] = { Tuning::TUNING_Q_RATE_ROLL_KPI, Tuning::TUNING_Q_RATE_ROLL_KD,
                                                                    Tuning::TUNING_Q_RATE_PITCH_KPI, Tuning::TUNING_Q_RATE_PITCH_KD};
static const Tuning::tuning_func tuning_set_q_rate_roll[] =       { Tuning::TUNING_Q_RATE_ROLL_KPI, Tuning::TUNING_Q_RATE_ROLL_KD };
static const Tuning::tuning_func tuning_set_q_rate_pitch[] =      { Tuning::TUNING_Q_RATE_PITCH_KPI, Tuning::TUNING_Q_RATE_PITCH_KD };

// macro to prevent getting the array length wrong
#define TUNING_ARRAY(v) ARRAY_SIZE(v), v

// list of tuning sets
const Tuning::tuning_set Tuning::tuning_sets[] = {
    { TUNING_SET_Q_RATE_ROLL_PITCH, TUNING_ARRAY(tuning_set_q_rate_roll_pitch) },
    { TUNING_SET_Q_RATE_ROLL,       TUNING_ARRAY(tuning_set_q_rate_roll) },
    { TUNING_SET_Q_RATE_PITCH,      TUNING_ARRAY(tuning_set_q_rate_pitch) }
};

/*
  table of tuning names
 */
const Tuning::tuning_name Tuning::tuning_names[] = {
    { TUNING_Q_RATE_ROLL_KPI, "Q_RateRollPI" },
    { TUNING_Q_RATE_ROLL_KD,  "Q_RateRollD" },
    { TUNING_Q_RATE_PITCH_KPI,"Q_RatePitchPI" },
    { TUNING_Q_RATE_PITCH_KD, "Q_RatePitchD" },
};

/*
  constructor
 */
Tuning::Tuning(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  handle selector switch input
*/
void Tuning::check_selector_switch(void)
{
    RC_Channel *selchan = RC_Channel::rc_channel(selector-1);
    if (selchan == nullptr) {
        return;
    }
    uint16_t selector_in = selchan->radio_in;
    if (selector_in >= 1700) {
        // high selector
        if (selector_start_ms == 0) {
            selector_start_ms = AP_HAL::millis();
        }
        uint32_t hold_time = AP_HAL::millis() - selector_start_ms;
        if (hold_time > 5000 && changed) {
            // save tune
            save_parameters();
            re_center();
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Tuning: Saved");
            AP_Notify::events.tune_save = 1;
            changed = false;
        }
    } else if (selector_in <= 1300) {
        // low selector
        if (selector_start_ms != 0) {
            uint32_t hold_time = AP_HAL::millis() - selector_start_ms;
            if (hold_time < 2000) {
                // re-center the value
                re_center();
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Tuning: recentered %s", get_tuning_name(current_parm));
            } else if (hold_time < 5000) {
                // change parameter
                next_parameter();
            }
        }
        selector_start_ms = 0;
    }
}

/*
  re-center the tuning value
 */
void Tuning::re_center(void)
{
    AP_Float *f = get_param_pointer(current_parm);
    if (f != nullptr) {
        center_value = f->get();
    }
    mid_point_wait = true;
}

/*
  check for changed tuning input
 */
void Tuning::check_input(void)
{
    if (channel <= 0 || selector <= 0 || parmset <= 0) {
        // disabled
        return;
    }

    // only adjust values at 10Hz
    uint32_t now = AP_HAL::millis();
    uint32_t dt_ms = now - last_check_ms;
    if (dt_ms < 100) {
        return;
    }
    last_check_ms = now;

    if (channel > hal.rcin->num_channels() ||
        selector > hal.rcin->num_channels()) {
        // not valid channels
        return;
    }

    // check for invalid range
    if (range < 1.1f) {
        range.set(1.1f);
    }
    
    check_selector_switch();

    if (selector_start_ms) {
        // no tuning while selector high
        return;
    }

    if (current_parm == TUNING_NONE) {
        next_parameter();
    }
    if (current_parm == TUNING_NONE) {
        return;
    }
    
    RC_Channel *chan = RC_Channel::rc_channel(channel-1);
    if (chan == nullptr) {
        return;
    }
    float chan_value = linear_interpolate(-1, 1, chan->radio_in, channel_min, channel_max);
    if (dt_ms > 500) {
        last_channel_value = chan_value;
    }

    if (fabsf(chan_value - last_channel_value) < 0.01) {
        // ignore changes of less than 1%
        return;
    }

    //hal.console->printf("chan_value %.2f last_channel_value %.2f\n", chan_value, last_channel_value);

    if (mid_point_wait) {
        if ((chan_value > 0 && last_channel_value > 0) ||
            (chan_value < 0 && last_channel_value < 0)) {
            // still waiting
            return;
        }
        // starting tuning
        mid_point_wait = false;
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Tuning: mid-point %s", get_tuning_name(current_parm));
        AP_Notify::events.tune_started = 1;
    }
    last_channel_value = chan_value;

    float new_value;
    if (chan_value > 0) {
        new_value = linear_interpolate(center_value, range*center_value, chan_value, 0, 1);
    } else {
        new_value = linear_interpolate(center_value/range, center_value, chan_value, -1, 0);
    }
    set_value(current_parm, new_value);
    Log_Write_Parameter_Tuning(new_value);
}


/*
  get a pointer to an AP_Float for a parameter, or NULL on fail
 */
AP_Float *Tuning::get_param_pointer(enum tuning_func parm)
{
    switch (parm) {
    case TUNING_RLL_P:
        return &plane.rollController.kP();

    case TUNING_RLL_I:
        return &plane.rollController.kI();

    case TUNING_RLL_D:
        return &plane.rollController.kD();

    case TUNING_RLL_FF:
        return &plane.rollController.kFF();

    case TUNING_PIT_P:
        return &plane.pitchController.kP();

    case TUNING_PIT_I:
        return &plane.pitchController.kI();

    case TUNING_PIT_D:
        return &plane.pitchController.kD();

    case TUNING_PIT_FF:
        return &plane.pitchController.kFF();
        
    default:
        break;
    }
        
    if (!plane.quadplane.available()) {
        // quadplane tuning options not available
        return nullptr;
    }
    
    switch(parm) {

    case TUNING_Q_RATE_ROLL_KPI:
        // use P for initial value when tuning PI
        return &plane.quadplane.attitude_control->get_rate_roll_pid().kP();

    case TUNING_Q_RATE_ROLL_KP:
        return &plane.quadplane.attitude_control->get_rate_roll_pid().kP();

    case TUNING_Q_RATE_ROLL_KI:
        return &plane.quadplane.attitude_control->get_rate_roll_pid().kI();

    case TUNING_Q_RATE_ROLL_KD:
        return &plane.quadplane.attitude_control->get_rate_roll_pid().kD();

    case TUNING_Q_RATE_PITCH_KPI:
        return &plane.quadplane.attitude_control->get_rate_pitch_pid().kP();

    case TUNING_Q_RATE_PITCH_KP:
        return &plane.quadplane.attitude_control->get_rate_pitch_pid().kP();

    case TUNING_Q_RATE_PITCH_KI:
        return &plane.quadplane.attitude_control->get_rate_pitch_pid().kI();

    case TUNING_Q_RATE_PITCH_KD:
        return &plane.quadplane.attitude_control->get_rate_pitch_pid().kD();

    case TUNING_Q_RATE_YAW_KPI:
        return &plane.quadplane.attitude_control->get_rate_yaw_pid().kP();

    case TUNING_Q_RATE_YAW_KP:
        return &plane.quadplane.attitude_control->get_rate_yaw_pid().kP();

    case TUNING_Q_RATE_YAW_KI:
        return &plane.quadplane.attitude_control->get_rate_yaw_pid().kI();

    case TUNING_Q_RATE_YAW_KD:
        return &plane.quadplane.attitude_control->get_rate_yaw_pid().kD();

    case TUNING_Q_ANG_ROLL_KP:
        return &plane.quadplane.attitude_control->get_angle_roll_p().kP();

    case TUNING_Q_ANG_PITCH_KP:
        return &plane.quadplane.attitude_control->get_angle_pitch_p().kP();

    case TUNING_Q_ANG_YAW_KP:
        return &plane.quadplane.attitude_control->get_angle_yaw_p().kP();

    case TUNING_Q_PXY_P:
        return &plane.quadplane.p_pos_xy.kP();

    case TUNING_Q_PZ_P:
        return &plane.quadplane.p_alt_hold.kP();

    case TUNING_Q_VXY_P:
        return &plane.quadplane.pi_vel_xy.kP();

    case TUNING_Q_VXY_I:
        return &plane.quadplane.pi_vel_xy.kI();

    case TUNING_Q_VZ_P:
        return &plane.quadplane.p_vel_z.kP();

    case TUNING_Q_AZ_P:
        return &plane.quadplane.pid_accel_z.kP();

    case TUNING_Q_AZ_I:
        return &plane.quadplane.pid_accel_z.kI();

    case TUNING_Q_AZ_D:
        return &plane.quadplane.pid_accel_z.kD();
        
    default:
        break;
    }
    return nullptr;
}


/*
  log a tuning change
 */
void Tuning::Log_Write_Parameter_Tuning(float value)
{
    struct log_ParameterTuning pkt_tune = {
        LOG_PACKET_HEADER_INIT(LOG_PARAMTUNE_MSG),
        time_us        : AP_HAL::micros64(),
        set            : parmset,
        parameter      : current_parm,
        tuning_value   : value,
        center_value   : center_value
    };

    plane.DataFlash.WriteBlock(&pkt_tune, sizeof(pkt_tune));
}

/*
  save parameters in the set
 */
void Tuning::save_parameters(void)
{
    enum tuning_sets set = (enum tuning_sets)parmset.get();
    for (uint8_t i=0; i<ARRAY_SIZE(tuning_sets); i++) {
        if (tuning_sets[i].set == set) {
            for (uint8_t p=0; p<tuning_sets[i].num_parms; p++) {
                save_value(tuning_sets[i].parms[p]);
            }
        }
    }
}

/*
  switch to the next parameter in the set
 */
void Tuning::next_parameter(void)
{
    enum tuning_sets set = (enum tuning_sets)parmset.get();
    for (uint8_t i=0; i<ARRAY_SIZE(tuning_sets); i++) {
        if (tuning_sets[i].set == set) {
            if (current_parm == TUNING_NONE) {
                current_parm_index = 0;
            } else {
                current_parm_index = (current_parm_index + 1) % tuning_sets[i].num_parms;
            }
            current_parm = tuning_sets[i].parms[current_parm_index];
            re_center();
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Tuning: started %s", get_tuning_name(current_parm));
            AP_Notify::events.tune_next = 1;
            break;
        }
    }
}

/*
  return a string representing a tuning parameter
 */
const char *Tuning::get_tuning_name(enum tuning_func parm)
{
    for (uint8_t i=0; i<ARRAY_SIZE(tuning_names); i++) {
        if (parm == tuning_names[i].parm) {
            return tuning_names[i].name;
        }
    }
    return "UNKNOWN";
}

/*
  save a parameter
 */
void Tuning::save_value(enum tuning_func parm)
{
    switch(parm) {
    // special handling of dual-parameters
    case TUNING_Q_RATE_ROLL_KPI:
        save_value(TUNING_Q_RATE_ROLL_KP);
        save_value(TUNING_Q_RATE_ROLL_KI);
        break;
    case TUNING_Q_RATE_PITCH_KPI:
        save_value(TUNING_Q_RATE_PITCH_KP);
        save_value(TUNING_Q_RATE_PITCH_KI);
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
void Tuning::set_value(enum tuning_func parm, float value)
{
    changed = true;
    switch(parm) {
    // special handling of dual-parameters
    case TUNING_Q_RATE_ROLL_KPI:
        set_value(TUNING_Q_RATE_ROLL_KP, value);
        set_value(TUNING_Q_RATE_ROLL_KI, value);
        break;
    case TUNING_Q_RATE_PITCH_KPI:
        set_value(TUNING_Q_RATE_PITCH_KP, value);
        set_value(TUNING_Q_RATE_PITCH_KI, value);
        break;
    default:
        AP_Float *f = get_param_pointer(parm);
        if (f != nullptr) {
            f->set(value);
        }
        break;
    }
}
