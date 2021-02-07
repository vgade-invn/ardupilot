#include "AP_TECS.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <stdio.h>
# define Debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
# define Debug(fmt, args ...)
#endif
//Debug("%.2f %.2f %.2f %.2f \n", var1, var2, var3, var4);

// table of user settable parameters
const AP_Param::GroupInfo AP_TECS::var_info[] = {

    // @Param: CLMB_MAX
    // @DisplayName: Maximum Climb Rate (metres/sec)
    // @Description: Maximum demanded climb rate. Do not set higher than the climb speed at THR_MAX at TRIM_ARSPD_CM when the battery is at low voltage. Reduce value if airspeed cannot be maintained on ascent. Increase value if throttle does not increase significantly to ascend.
    // @Increment: 0.1
    // @Range: 0.1 20.0
    // @User: Standard
    AP_GROUPINFO("CLMB_MAX",    0, AP_TECS, _maxClimbRate, 5.0f),

    // @Param: SINK_MIN
    // @DisplayName: Minimum Sink Rate (metres/sec)
    // @Description: Minimum sink rate when at THR_MIN and TRIM_ARSPD_CM.
    // @Increment: 0.1
    // @Range: 0.1 10.0
    // @User: Standard
    AP_GROUPINFO("SINK_MIN",    1, AP_TECS, _minSinkRate, 2.0f),

    // @Param: TIME_CONST
    // @DisplayName: Controller time constant (sec)
    // @Description: Time constant of the TECS control algorithm. Small values make faster altitude corrections but can cause overshoot and aggressive behavior.
    // @Range: 3.0 10.0
    // @Increment: 0.2
    // @User: Advanced
    AP_GROUPINFO("TIME_CONST",  2, AP_TECS, _timeConst, 5.0f),

    // @Param: THR_DAMP
    // @DisplayName: Controller throttle damping
    // @Description: Damping gain for throttle demand loop. Slows the throttle response to correct for speed and height oscillations.
    // @Range: 0.1 1.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("THR_DAMP",    3, AP_TECS, _thrDamp, 0.5f),

    // @Param: INTEG_GAIN
    // @DisplayName: Controller integrator
    // @Description: Integrator gain to trim out long-term speed and height errors.
    // @Range: 0.0 0.5
    // @Increment: 0.02
    // @User: Advanced
    AP_GROUPINFO("INTEG_GAIN", 4, AP_TECS, _integGain, 0.1f),

    // @Param: VERT_ACC
    // @DisplayName: Vertical Acceleration Limit (metres/sec^2)
    // @Description: Maximum vertical acceleration used to correct speed or height errors.
    // @Range: 1.0 10.0
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("VERT_ACC",  5, AP_TECS, _vertAccLim, 7.0f),

    // @Param: HGT_OMEGA
    // @DisplayName: Height complementary filter frequency (radians/sec)
    // @Description: This is the cross-over frequency of the complementary filter used to fuse vertical acceleration and baro alt to obtain an estimate of height rate and height.
    // @Range: 1.0 5.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("HGT_OMEGA", 6, AP_TECS, _hgtCompFiltOmega, 3.0f),

    // @Param: SPD_OMEGA
    // @DisplayName: Speed complementary filter frequency (radians/sec)
    // @Description: This is the cross-over frequency of the complementary filter used to fuse longitudinal acceleration and airspeed to obtain a lower noise and lag estimate of airspeed.
    // @Range: 0.5 2.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("SPD_OMEGA", 7, AP_TECS, _spdCompFiltOmega, 2.0f),

    // @Param: RLL2THR
    // @DisplayName: Bank angle compensation gain
    // @Description: Gain from bank angle to throttle to compensate for loss of airspeed from drag in turns. Set to approximately 10x the sink rate in m/s caused by a 45-degree turn. High efficiency models may need less while less efficient aircraft may need more. Should be tuned in an automatic mission with waypoints and turns greater than 90 degrees. Tune with PTCH2SV_RLL and KFF_RDDRMIX to achieve constant airspeed, constant altitude turns.
    // @Range: 5.0 30.0
    // @Increment: 1.0
    // @User: Advanced
    AP_GROUPINFO("RLL2THR",  8, AP_TECS, _rollComp, 10.0f),

    // @Param: SPDWEIGHT
    // @DisplayName: Weighting applied to speed control
    // @Description: Mixing of pitch and throttle correction for height and airspeed errors. Pitch controls altitude and throttle controls airspeed if set to 0. Pitch controls airspeed and throttle controls altitude if set to 2 (good for gliders). Blended if set to 1.
    // @Range: 0.0 2.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SPDWEIGHT", 9, AP_TECS, _spdWeight, 1.0f),

    // @Param: PTCH_DAMP
    // @DisplayName: Controller pitch damping
    // @Description: Damping gain for pitch control from TECS control.  Increasing may correct for oscillations in speed and height, but too much may cause additional oscillation and degraded control.
    // @Range: 0.1 1.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("PTCH_DAMP", 10, AP_TECS, _ptchDamp, 0.0f),

    // @Param: SINK_MAX
    // @DisplayName: Maximum Descent Rate (metres/sec)
    // @Description: Maximum demanded descent rate. Do not set higher than the vertical speed the aircraft can maintain at THR_MIN, TECS_PITCH_MIN, and ARSPD_FBW_MAX.
    // @Increment: 0.1
    // @Range: 0.0 20.0
    // @User: Standard
    AP_GROUPINFO("SINK_MAX",  11, AP_TECS, _maxSinkRate, 5.0f),

    // @Param: LAND_ARSPD
    // @DisplayName: Airspeed during landing approach (m/s)
    // @Description: When performing an autonomus landing, this value is used as the goal airspeed during approach.  Note that this parameter is not useful if your platform does not have an airspeed sensor (use TECS_LAND_THR instead).  If negative then this value is not used during landing.
    // @Range: -1 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("LAND_ARSPD", 12, AP_TECS, _landAirspeed, -1),

    // @Param: LAND_THR
    // @DisplayName: Cruise throttle during landing approach (percentage)
    // @Description: Use this parameter instead of LAND_ARSPD if your platform does not have an airspeed sensor.  It is the cruise throttle during landing approach.  If this value is negative then it is disabled and TECS_LAND_ARSPD is used instead.
    // @Range: -1 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("LAND_THR", 13, AP_TECS, _landThrottle, -1),

    // @Param: LAND_SPDWGT
    // @DisplayName: Weighting applied to speed control during landing.
    // @Description: Same as SPDWEIGHT parameter, with the exception that this parameter is applied during landing flight stages.  A value closer to 2 will result in the plane ignoring height error during landing and our experience has been that the plane will therefore keep the nose up -- sometimes good for a glider landing (with the side effect that you will likely glide a ways past the landing point).  A value closer to 0 results in the plane ignoring speed error -- use caution when lowering the value below 1 -- ignoring speed could result in a stall. Values between 0 and 2 are valid values for a fixed landing weight. When using -1 the weight will be scaled during the landing. At the start of the landing approach it starts with TECS_SPDWEIGHT and scales down to 0 by the time you reach the land point. Example: Halfway down the landing approach you'll effectively have a weight of TECS_SPDWEIGHT/2.
    // @Range: -1.0 2.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("LAND_SPDWGT", 14, AP_TECS, _spdWeightLand, -1.0f),

    // @Param: PITCH_MAX
    // @DisplayName: Maximum pitch in auto flight
    // @Description: Overrides LIM_PITCH_MAX in automatic throttle modes to reduce climb rates. Uses LIM_PITCH_MAX if set to 0. For proper TECS tuning, set to the angle that the aircraft can climb at TRIM_ARSPD_CM and THR_MAX.
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PITCH_MAX", 15, AP_TECS, _pitch_max, 15),

    // @Param: PITCH_MIN
    // @DisplayName: Minimum pitch in auto flight
    // @Description: Overrides LIM_PITCH_MIN in automatic throttle modes to reduce descent rates. Uses LIM_PITCH_MIN if set to 0. For proper TECS tuning, set to the angle that the aircraft can descend at without overspeeding.
    // @Range: -45 0
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PITCH_MIN", 16, AP_TECS, _pitch_min, 0),

    // @Param: LAND_SINK
    // @DisplayName: Sink rate for final landing stage
    // @Description: The sink rate in meters/second for the final stage of landing.
    // @Range: 0.0 2.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("LAND_SINK", 17, AP_TECS, _land_sink, 0.25f),

    // @Param: LAND_TCONST
    // @DisplayName: Land controller time constant (sec)
    // @Description: This is the time constant of the TECS control algorithm when in final landing stage of flight. It should be smaller than TECS_TIME_CONST to allow for faster flare
    // @Range: 1.0 5.0
    // @Increment: 0.2
    // @User: Advanced
    AP_GROUPINFO("LAND_TCONST", 18, AP_TECS, _landTimeConst, 2.0f),

    // @Param: LAND_DAMP
    // @DisplayName: Controller sink rate to pitch gain during flare
    // @Description: This is the sink rate gain for the pitch demand loop when in final landing stage of flight. It should be larger than TECS_PTCH_DAMP to allow for better sink rate control during flare.
    // @Range: 0.1 1.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("LAND_DAMP", 19, AP_TECS, _landDamp, 0.5f),

    // @Param: LAND_PMAX
    // @DisplayName: Maximum pitch during final stage of landing
    // @Description: This limits the pitch used during the final stage of automatic landing. During the final landing stage most planes need to keep their pitch small to avoid stalling. A maximum of 10 degrees is usually good. A value of zero means to use the normal pitch limits.
    // @Range: -5 40
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LAND_PMAX", 20, AP_TECS, _land_pitch_max, 10),

    // @Param: APPR_SMAX
    // @DisplayName: Sink rate max for landing approach stage
    // @Description: The sink rate max for the landing approach stage of landing. This will need to be large for steep landing approaches especially when using reverse thrust. If 0, then use TECS_SINK_MAX.
    // @Range: 0.0 20.0
    // @Units: m/s
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("APPR_SMAX", 21, AP_TECS, _maxSinkRate_approach, 0),

    // @Param: LAND_SRC
    // @DisplayName: Land sink rate change
    // @Description: When zero, the flare sink rate (TECS_LAND_SINK) is a fixed sink demand. With this enabled the flare sinkrate will increase/decrease the flare sink demand as you get further beyond the LAND waypoint. Has no effect before the waypoint. This value is added to TECS_LAND_SINK proportional to distance traveled after wp. With an increasing sink rate you can still land in a given distance if you're traveling too fast and cruise passed the land point. A positive value will force the plane to land sooner proportional to distance passed land point. A negative number will tell the plane to slowly climb allowing for a pitched-up stall landing. Recommend 0.2 as initial value.
    // @Range: -2.0 2.0
    // @Units: m/s/m
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("LAND_SRC", 22, AP_TECS, _land_sink_rate_change, 0),

    // @Param: LAND_TDAMP
    // @DisplayName: Controller throttle damping when landing
    // @Description: This is the damping gain for the throttle demand loop during and auto-landing. Same as TECS_THR_DAMP but only in effect during an auto-land. Increase to add damping to correct for oscillations in speed and height. When set to 0 landing throttle damp is controlled by TECS_THR_DAMP.
    // @Range: 0.1 1.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("LAND_TDAMP", 23, AP_TECS, _land_throttle_damp, 0),

    // @Param: LAND_IGAIN
    // @DisplayName: Controller integrator during landing
    // @Description: This is the integrator gain on the control loop during landing. When set to 0 then TECS_INTEG_GAIN is used. Increase to increase the rate at which speed and height offsets are trimmed out. Typically values lower than TECS_INTEG_GAIN work best
    // @Range: 0.0 0.5
    // @Increment: 0.02
    // @User: Advanced
    AP_GROUPINFO("LAND_IGAIN", 24, AP_TECS, _integGain_land, 0),

    // @Param: TKOFF_IGAIN
    // @DisplayName: Controller integrator during takeoff
    // @Description: This is the integrator gain on the control loop during takeoff. When set to 0 then TECS_INTEG_GAIN is used. Increase to increase the rate at which speed and height offsets are trimmed out. Typically values higher than TECS_INTEG_GAIN work best
    // @Range: 0.0 0.5
    // @Increment: 0.02
    // @User: Advanced
    AP_GROUPINFO("TKOFF_IGAIN", 25, AP_TECS, _integGain_takeoff, 0),

    // @Param: LAND_PDAMP
    // @DisplayName: Pitch damping gain when landing
    // @Description: This is the damping gain for the pitch demand loop during landing. Increase to add damping  to correct for oscillations in speed and height. If set to 0 then TECS_PTCH_DAMP will be used instead.
    // @Range: 0.1 1.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("LAND_PDAMP", 26, AP_TECS, _land_pitch_damp, 0),

    // @Param: SYNAIRSPEED
    // @DisplayName: Enable the use of synthetic airspeed
    // @Description: This enable the use of synthetic airspeed for aircraft that don't have a real airspeed sensor. This is useful for development testing where the user is aware of the considerable limitations of the synthetic airspeed system, such as very poor estimates when a wind estimate is not accurate. Do not enable this option unless you fully understand the limitations of a synthetic airspeed estimate.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("SYNAIRSPEED", 27, AP_TECS, _use_synthetic_airspeed, 0),

    // @Param: OPTIONS
    // @DisplayName: Extra TECS options
    // @Description: This allows the enabling of special features in the speed/height controller.
    // @Bitmask: 0:GliderOnly,1:SmoothSpeed
    // @User: Advanced
    AP_GROUPINFO("OPTIONS", 28, AP_TECS, _options, 0),

    // @Param: PTCH_FF_V0
    // @DisplayName: Baseline airspeed for pitch feed-forward.
    // @Description: This parameter sets the airspeed at which no feed-forward is applied between demanded airspeed and pitch. It should correspond to the airspeed in metres per second at which the plane glides at neutral pitch including STAB_PITCH_DOWN.
    // @Range: 5.0 50.0
    // @User: Advanced
    AP_GROUPINFO("PTCH_FF_V0", 29, AP_TECS, _pitch_ff_v0, 12.0),

    // @Param: PTCH_FF_K
    // @DisplayName: Gain for pitch feed-forward.
    // @Description: This parameter sets the gain between demanded airspeed and pitch. It has units of radians per metre per second and should generally be negative. A good starting value is -0.04 for gliders and -0.08 for draggy airframes. The default (0.0) disables this feed-forward.
    // @Range: -5.0 0.0
    // @User: Advanced
    AP_GROUPINFO("PTCH_FF_K", 30, AP_TECS, _pitch_ff_k, 0.0),
    
    // @Param: LAND_PTRIM
    // @DisplayName: Pitch angle for level flight in landing confiuration
    // @Description: This sets the pitch angle required to fly straight and level with flaps and gear in the landing configuration. It is used to calculate the lower pitch limit applied during landing up antil the flare. This can be set to the average value of the AOA.AOA log data taken from a landing approach.
    // @Range: -10 15
    // @Units: deg
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LAND_PTRIM", 31, AP_TECS, _land_pitch_trim, 0),

    // @Param: FLARE_HGT
    // @DisplayName: Flare holdoff height
    // @Description: When height above ground is below this, the sink rate will be held at TECS_LAND_SINK. Use this to perform a hold-of manoeuvre when combined with small values for TECS_LAND_SINK.
    // @Range: -10 15
    // @Units: deg
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("FLARE_HGT", 32, AP_TECS, _flare_holdoff_hgt, 1.0f),

    // @Param: HDEM_TCONST
    // @DisplayName: Height Demand Time Constant
    // @Description: This sets the time constant of the low pass filter that is applied to the height demand input when bit 1 of TECS_OPTIONS is not selected.
    // @Range: -1.0 5.0
    // @Units: s
    // @Increment: 0.2
    // @User: Advanced
    AP_GROUPINFO("HDEM_TCONST", 33, AP_TECS, _hgt_dem_tconst, 2.0f),

    // @Param: TRIM_AOA
    // @DisplayName: Trim angle of attack
    // @Description: Set to the pitch angle in level flight at the trim airspeed.
    // @Range: -5.0 15.0
    // @Units: deg
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("TRIM_AOA", 34, AP_TECS, _trim_aoa, 3.0f),

    // @Param: ASPD_SLEW
    // @DisplayName: Airspeed slew rate factor
    // @Description: Used to speed up or slow down the internally calculated airspeed demand slew rate limit. Bigger values make airspeed change faster.
    // @Range: 0.1 2.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("ASPD_SLEW", 35, AP_TECS, _aspd_slew_factor, 1.0f),

    // @Param: TC_E2T_EXP
    // @DisplayName: Time constant altitude scaling exponent
    // @Description: Exponent used to calculate the increase in time constant with TAS to EAS ratio where the control loop tine constant = TECS_TCONST x EAS2TAS ^ TECS_TC_E2T_EXP
    // @Range: 0.5 1.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("TC_E2T_EXP", 36, AP_TECS, _tconst_exponent, 0.7f),

    AP_GROUPEND
};

/*
 *  Written by Paul Riseborough 2013 to provide:
 *  - Combined control of speed and height using throttle to control
 *    total energy and pitch angle to control exchange of energy between
 *    potential and kinetic.
 *    Selectable speed or height priority modes when calculating pitch angle
 *  - Fallback mode when no airspeed measurement is available that
 *    sets throttle based on height rate demand and switches pitch angle control to
 *    height priority
 *  - Underspeed protection that demands maximum throttle and switches pitch angle control
 *    to speed priority mode
 *  - Relative ease of tuning through use of intuitive time constant, integrator and damping gains and the use
 *    of easy to measure aircraft performance data
 *
 */

void AP_TECS::update_50hz(void)
{
    // Implement third order complementary filter for height and height rate
    // estimated height rate = _climb_rate
    // estimated height above field elevation  = _height
    // Reference Paper :
    // Optimizing the Gains of the Baro-Inertial Vertical Channel
    // Widnall W.S, Sinha P.K,
    // AIAA Journal of Guidance and Control, 78-1307R

    _update_airspeed_conversions();

    /*
      if we have a vertical position estimate from the EKF then use
      it, otherwise use barometric altitude
     */
    _ahrs.get_relative_position_D_home(_height);
    _height *= -1.0f;

    // Calculate time in seconds since last update
    uint64_t now = AP_HAL::micros64();
    float DT = (now - _update_50hz_last_usec) * 1.0e-6f;
    if (DT > 1.0f) {
        _climb_rate = 0.0f;
        _height_filter.dd_height = 0.0f;
        _height_filter.height = _height;
        DT = 0.1f; // when first starting TECS, use the most likely value
    }
    _update_50hz_last_usec = now;

    // Use inertial nav verical velocity and height if available
    Vector3f velned;
    if (_ahrs.get_velocity_NED(velned)) {
        // if possible use the EKF vertical velocity
        _climb_rate = -velned.z;
    } else {
        /*
          use a complimentary filter to calculate climb_rate. This is
          designed to minimise lag
         */
        const float baro_alt = AP::baro().get_altitude();
        // Get height acceleration
        float hgt_ddot_mea = -(_ahrs.get_accel_ef().z + GRAVITY_MSS);
        // Perform filter calculation using backwards Euler integration
        // Coefficients selected to place all three filter poles at omega
        float omega2 = _hgtCompFiltOmega*_hgtCompFiltOmega;
        float hgt_err = baro_alt - _height_filter.height;
        float integ1_input = hgt_err * omega2 * _hgtCompFiltOmega;

        _height_filter.dd_height += integ1_input * DT;

        float integ2_input = _height_filter.dd_height + hgt_ddot_mea + hgt_err * omega2 * 3.0f;

        _climb_rate += integ2_input * DT;

        float integ3_input = _climb_rate + hgt_err * _hgtCompFiltOmega * 3.0f;

        _height_filter.height += integ3_input*DT;
    }

    // Update and average speed rate of change
    const Matrix3f &rotMat = _ahrs.get_rotation_body_to_ned();
    // Unit vector in body frame aligned with average wind relative velocity vector
    const Vector3f vel_unit_wrt_wind = Vector3f(cosf(radians(_trim_aoa)), 0.0f, sinf(radians(_trim_aoa)));
    // calculate the measured rate of change of speed along the wind relative velocity vector
    Vector3f earth_frame_vdot = _ahrs.get_accel_ef();
    earth_frame_vdot.z += GRAVITY_MSS;
    const Vector3f body_frame_vdot = rotMat.mul_transpose(earth_frame_vdot);
    const float vel_dot_raw = body_frame_vdot * vel_unit_wrt_wind;
    // Place filter time constants a factor of 5 above and 10 below the TECS time constant
    const float alpha_hpf = _DT / (5.0f * timeConstant());
    const float alpha_lpf = 50.0f * alpha_hpf;
    _vel_dot_lpf_out = (1.0f - alpha_lpf) * _vel_dot_lpf_out + alpha_lpf * vel_dot_raw;
    _vel_dot_hpf_out += _vel_dot_lpf_out - _vel_dot_hpf_in;
    _vel_dot_hpf_in = _vel_dot_lpf_out;
    _vel_dot_hpf_out *= (1.0f - alpha_hpf);

}

void AP_TECS::_update_speed(float load_factor)
{
    // Calculate time in seconds since last update
    uint64_t now = AP_HAL::micros64();
    float DT = (now - _update_speed_last_usec) * 1.0e-6f;
    _update_speed_last_usec = now;

    _update_airspeed_conversions();

    // Reset states of time since last update is too large
    if (DT > 1.0f) {
        _reset_airspeed_states();
        DT = 0.1f;
    }

    _EASmax   = aparm.airspeed_max;
    _EASmin   = aparm.airspeed_min;

    if (aparm.stall_prevention) {
        // when stall prevention is active we raise the mimimum
        // airspeed based on aerodynamic load factor
        _EASmin *= safe_sqrt(load_factor);
    }

    if (_EASmax < _EASmin) {
        _EASmax = _EASmin;
    }

    // Get airspeed or default to halfway between min and max if
    // airspeed is not being used and set speed rate to zero
    bool use_airspeed = _use_synthetic_airspeed_once || _use_synthetic_airspeed.get() || _ahrs.airspeed_sensor_enabled();
    if (!use_airspeed || !_ahrs.airspeed_estimate(_EAS)) {
        // If no airspeed available use average of min and max
        _EAS = 0.5f * (aparm.airspeed_min.get() + (float)aparm.airspeed_max.get());
    }

    // Implement a second order complementary filter to obtain a
    // smoothed airspeed estimate
    // airspeed estimate is held in _TAS_state
    float aspdErr = (_EAS * _EAS2TAS) - _TAS_state;
    float integDTAS_input = aspdErr * _spdCompFiltOmega * _spdCompFiltOmega;
    // Prevent state from winding up
    if (_TAS_state < 3.1f) {
        integDTAS_input = MAX(integDTAS_input , 0.0f);
    }
    _integDTAS_state = _integDTAS_state + integDTAS_input * DT;
    float TAS_input = _integDTAS_state + _vel_dot_lpf_out + aspdErr * _spdCompFiltOmega * 1.4142f;
    _TAS_state = _TAS_state + TAS_input * DT;
    // limit the airspeed to a minimum of 3 m/s
    _TAS_state = MAX(_TAS_state, 3.0f);

}

void AP_TECS::_update_speed_demand(void)
{
    // Set the airspeed demand to the minimum value if an underspeed condition exists
    // or a bad descent condition exists
    // This will minimise the rate of descent resulting from an engine failure,
    // enable the maximum climb rate to be achieved and prevent continued full power descent
    // into the ground due to an unachievable airspeed value
    if ((_flags.badDescent) || (_flags.underspeed))
    {
        _EAS_dem = _EASmin;
    }

    // Constrain speed demand
    _EAS_dem = constrain_float(_EAS_dem, _EASmin, _EASmax);

    // calculate velocity rate limits based on physical performance limits
    // provision to use a different rate limit if bad descent or underspeed condition exists
    float velRateMin, velRateMax;
    if (_flags.is_gliding) {
        // The rate of acceleration is a function of the how much further the flight path angle
        // can be lowered or raised.
        velRateMax = GRAVITY_MSS * sinf((_pitch_dem - _PITCHminf));
        velRateMin = GRAVITY_MSS * sinf((_pitch_dem - _PITCHmaxf));
    } else {
        // Use 50% of maximum energy rate to allow margin for total energy contgroller
        velRateMax = 0.5f * _STEdot_max / _TAS_state;
        velRateMin = 0.5f * _STEdot_min / _TAS_state;
    }
    velRateMax *= _TAS2EAS * _aspd_slew_factor;
    velRateMin *= _TAS2EAS * _aspd_slew_factor;

    // Apply rate limit
    const float EAS_dem_previous = _EAS_dem_rlim;
    if ((_EAS_dem - EAS_dem_previous) > (velRateMax * _DT))
    {
        _EAS_dem_rlim = EAS_dem_previous + velRateMax * _DT;
    }
    else if ((_EAS_dem - EAS_dem_previous) < (velRateMin * _DT))
    {
        _EAS_dem_rlim = EAS_dem_previous + velRateMin * _DT;
    }
    else
    {
        _EAS_dem_rlim = _EAS_dem;
    }

    // gliders need a smoother speed demand to prevent unwanted pitch transients
    if (_flags.is_gliding && _options & OPTION_SMOOTH_SPEED) {
        const float tconst = timeConstant();
        const float filt_coef = _DT / tconst;
        _EAS_dem_lpf_1 = (1.0f - filt_coef) * _EAS_dem_lpf_1 + filt_coef * _EAS_dem_rlim;
        const float EAS_dem_lpf_2_prev = _EAS_dem_lpf_2;
        _EAS_dem_lpf_2 = (1.0f - filt_coef) * _EAS_dem_lpf_2 + filt_coef * _EAS_dem_lpf_1;
        _TAS_rate_dem = _EAS2TAS * (_EAS_dem_lpf_2 - EAS_dem_lpf_2_prev) / tconst;
   } else {
        _EAS_dem_lpf_2 = _EAS_dem_lpf_1 = _EAS_dem_rlim;
        _TAS_rate_dem = _EAS2TAS * (_EAS_dem_rlim - EAS_dem_previous) / _DT;
    }
    _TAS_dem =  _EAS2TAS * constrain_float(_EAS_dem_lpf_2, _EASmin, _EASmax);
}

void AP_TECS::_update_height_demand(void)
{
    if (!_landing.is_flaring()) {
        // Apply 2 point moving average to demanded height
        const float hgt_dem = 0.5f * (_hgt_dem_in + _hgt_dem_in_prev);
        _hgt_dem_in_prev = _hgt_dem_in;

        float max_sink_rate = _maxSinkRate;
        if (_maxSinkRate_approach > 0 && _flags.is_doing_auto_land) {
            // special sink rate for approach to accommodate steep slopes and reverse thrust.
            // A special check must be done to see if we're LANDing on approach but also if
            // we're in that tiny window just starting NAV_LAND but still in NORMAL mode. If
            // we have a steep slope with a short approach we'll want to allow acquiring the
            // glide slope right away.
            max_sink_rate = _maxSinkRate_approach;
        }

        // Limit height rate of change
        if ((hgt_dem - _hgt_dem_rate_ltd) > (_maxClimbRate * _DT))
        {
            _hgt_dem_rate_ltd = _hgt_dem_rate_ltd + _maxClimbRate * _DT;
        }
        else if ((hgt_dem - _hgt_dem_rate_ltd) < (-max_sink_rate * _DT))
        {
            _hgt_dem_rate_ltd = _hgt_dem_rate_ltd - max_sink_rate * _DT;
        } else {
            _hgt_dem_rate_ltd = hgt_dem;
        }

        // Apply 2 second first order lag to height demand and compensate for lag when commencing height
        // control after takeoff to prevent plane pushing nose to level before climbing again. Post takeoff
        // compensation offset is decayed using the same time constant as the height demand filter.
        const float coef = MIN(_DT / _hgt_dem_tconst, 1.0f);
        _post_TO_hgt_offset *= (1.0f - coef);
        const float lpf_input = (_hgt_dem_rate_ltd + _post_TO_hgt_offset);
        _hgt_rate_dem = (lpf_input - _hgt_dem_lpf) / _hgt_dem_tconst;
        _hgt_dem_lpf = lpf_input * coef + (1.0f - coef) * _hgt_dem_lpf;

        _hgt_dem = _hgt_dem_lpf;

        // during approach compensate for height filter lag
        if (_flags.is_doing_auto_land) {
            _hgt_dem += _hgt_dem_tconst * _hgt_rate_dem;
        }

    } else {
        // when flaring force height rate demand to the
        // configured sink rate and adjust the demanded height to
        // be kinematically consistent with the height rate.

        // set all height filter states to current height to prevent large pith transients if flare is aborted
        _hgt_dem_lpf      = _height;
        _hgt_dem_rate_ltd = _height;
        _hgt_dem_in_prev  = _height;

        if (_flare_counter == 0) {
            _flare_hgt_dem_adj = _hgt_dem; 
            _flare_hgt_dem_ideal = _hgt_afe; 
            _hgt_at_start_of_flare = _hgt_afe;
            _hgt_rate_at_flare_entry = _climb_rate;

            // adjust flare height controller integrator so that at flare entry the pitch
            const float K_P = 1.0f / _landTimeConst;
            const float hgt_offset = _flare_hgt_dem_adj - _flare_hgt_dem_ideal;
            _hgt_rate_err_integ += hgt_offset * K_P;
        }

        // adjust the flare sink rate to increase/decrease as your travel further beyond the land wp
        float land_sink_rate_adj = _land_sink + _land_sink_rate_change*_distance_beyond_land_wp;

        // bring it in linearly with height
        float p;
        if (_hgt_at_start_of_flare > _flare_holdoff_hgt) {
            p = constrain_float((_hgt_at_start_of_flare - _hgt_afe) / (_hgt_at_start_of_flare - _flare_holdoff_hgt), 0.0f, 1.0f);
        } else {
            p = 1.0f;
        }
        _hgt_rate_dem = _hgt_rate_at_flare_entry * (1.0f - p) - land_sink_rate_adj * p;

        _flare_counter++;

        _flare_hgt_dem_ideal += _DT * _hgt_rate_dem; // the ideal height profile to follow
        _flare_hgt_dem_adj   += _DT * _hgt_rate_dem; // the demanded height profile that includes the pre-flare height tracking offset

        // fade across to the ideal height profile
        _hgt_dem = _flare_hgt_dem_adj * (1.0f - p) + _flare_hgt_dem_ideal * p;

        // correct for offset between height above ground and height above datum used by control loops
        _hgt_dem += (_hgt_afe - _height);
    }
}

void AP_TECS::_detect_underspeed(void)
{
    // see if we can clear a previous underspeed condition. We clear
    // it if we are now more than 15% above min speed, and haven't
    // been below min speed for at least 3 seconds.
    if (_flags.underspeed &&
        _TAS_state >= _EASmin * 1.15f &&
        AP_HAL::millis() - _underspeed_start_ms > 3000U) {
        _flags.underspeed = false;
    }

    if (_flight_stage == AP_Vehicle::FixedWing::FLIGHT_VTOL) {
        _flags.underspeed = false;
    } else if (((_TAS_state < _EASmin * 0.9f) &&
            (_throttle_dem >= _THRmaxf * 0.95f) &&
            !_landing.is_flaring()) ||
            ((_height < _hgt_dem) && _flags.underspeed))
    {
        _flags.underspeed = true;
        if (_TAS_state < _EASmin * 0.9f) {
            // reset start time as we are still underspeed
            _underspeed_start_ms = AP_HAL::millis();
        }
    }
    else
    {
        // this clears underspeed if we reach our demanded height and
        // we are either below 95% throttle or we above 90% of min
        // airspeed
        _flags.underspeed = false;
    }
}

void AP_TECS::_update_energies(void)
{
    // Calculate specific energy demands
    _SPE_dem = _hgt_dem * GRAVITY_MSS;
    _SKE_dem = 0.5f * _TAS_dem * _TAS_dem;

    // Calculate specific energy rate demands
    _SKEdot_dem = _TAS_state * _TAS_rate_dem;

    // Calculate specific energy
    _SPE_est = _height * GRAVITY_MSS;
    _SKE_est = 0.5f * _TAS_state * _TAS_state;

    // Calculate specific energy rate
    _SPEdot = _climb_rate * GRAVITY_MSS;
    _SKEdot = _TAS_state * _vel_dot_hpf_out;

}

/*
  current time constant. It is lower in landing to try to give a precise approach
 */
float AP_TECS::timeConstant(void) const
{
    const float alt_scale = powf(_EAS2TAS, _tconst_exponent);
    if (_flags.is_doing_auto_land) {
        if (_landTimeConst < 0.1f) {
            return 0.1f * alt_scale;
        }
        return _landTimeConst;
    }
    if (_timeConst < 0.1f) {
        return 0.1f * alt_scale;
    }
    return _timeConst * alt_scale;
}

/*
  calculate throttle demand - airspeed enabled case
 */
void AP_TECS::_update_throttle_with_airspeed(void)
{
    // Calculate limits to be applied to potential energy error to prevent over or underspeed occurring due to large height errors
    float SPE_err_max = 0.5f * _EASmax * _EASmax - _SKE_dem;
    float SPE_err_min = 0.5f * _EASmin * _EASmin - _SKE_dem;

    if (_flight_stage == AP_Vehicle::FixedWing::FLIGHT_VTOL) {
        /*
          when we are in a VTOL state then we ignore potential energy
          errors as we have vertical motors that interfere with the
          total energy calculation.
         */
        SPE_err_max = SPE_err_min = 0;
    }
    
    // Calculate total energy error
    _STE_error = constrain_float((_SPE_dem - _SPE_est), SPE_err_min, SPE_err_max) + _SKE_dem - _SKE_est;
    float STEdot_dem = constrain_float((_SPEdot_dem + _SKEdot_dem), _STEdot_min, _STEdot_max);
    float STEdot_error = STEdot_dem - _SPEdot - _SKEdot;

    // Apply 0.5 second first order filter to STEdot_error
    // This is required to remove accelerometer noise from the  measurement
    const float filt_coef = 2.0f * _DT;
    STEdot_error = filt_coef * STEdot_error + (1.0f - filt_coef) * _STEdotErrLast;
    _STEdotErrLast = STEdot_error;

    // Calculate throttle demand
    // If underspeed condition is set, then demand full throttle
    if (_flags.underspeed)
    {
        _throttle_dem = 1.0f;
    }
    else if (_flags.is_gliding)
    {
        _throttle_dem = 0.0f;
    }
    else
    {
        // Calculate gain scaler from specific energy error to throttle
        // (_STEdot_max - _STEdot_min) / (_THRmaxf - _THRminf) is the derivative of STEdot wrt throttle measured across the max allowed throttle range.
        float K_STE2Thr = 1 / (timeConstant() * (_STEdot_max - _STEdot_min) / (_THRmaxf - _THRminf));

        // Calculate feed-forward throttle
        float ff_throttle = 0;
        float nomThr = aparm.throttle_cruise * 0.01f;
        const Matrix3f &rotMat = _ahrs.get_rotation_body_to_ned();
        // Use the demanded rate of change of total energy as the feed-forward demand, but add
        // additional component which scales with (1/cos(bank angle) - 1) to compensate for induced
        // drag increase during turns.
        float cosPhi = sqrtf((rotMat.a.y*rotMat.a.y) + (rotMat.b.y*rotMat.b.y));
        STEdot_dem = STEdot_dem + _rollComp * (1.0f/constrain_float(cosPhi * cosPhi , 0.1f, 1.0f) - 1.0f);
        ff_throttle = nomThr + STEdot_dem / (_STEdot_max - _STEdot_min) * (_THRmaxf - _THRminf);

        // Calculate PD + FF throttle
        float throttle_damp = _thrDamp;
        if (_flags.is_doing_auto_land && !is_zero(_land_throttle_damp)) {
            throttle_damp = _land_throttle_damp;
        }
        _throttle_dem = (_STE_error + STEdot_error * throttle_damp) * K_STE2Thr + ff_throttle;

        // Constrain throttle demand
        _throttle_dem = constrain_float(_throttle_dem, _THRminf, _THRmaxf);

        float THRminf_clipped_to_zero = constrain_float(_THRminf, 0, _THRmaxf);

        // Calculate integrator state upper and lower limits
        // Set to a value that will allow 0.1 (10%) throttle saturation to allow for noise on the demand
        // Additionally constrain the integrator state amplitude so that the integrator comes off limits faster.
        float maxAmp = 0.5f*(_THRmaxf - THRminf_clipped_to_zero);
        float integ_max = constrain_float((_THRmaxf - _throttle_dem + 0.1f),-maxAmp,maxAmp);
        float integ_min = constrain_float((_THRminf - _throttle_dem - 0.1f),-maxAmp,maxAmp);

        // Calculate integrator state, constraining state
        // Set integrator to a max throttle value during climbout
        _integTHR_state = _integTHR_state + (_STE_error * _get_i_gain()) * _DT * K_STE2Thr;
        if (_flight_stage == AP_Vehicle::FixedWing::FLIGHT_TAKEOFF || _flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND)
        {
            if (!_flags.reached_speed_takeoff) {
                // ensure we run at full throttle until we reach the target airspeed
                _throttle_dem = MAX(_throttle_dem, _THRmaxf - _integTHR_state);
            }
            _integTHR_state = integ_max;
        }
        else
        {
            _integTHR_state = constrain_float(_integTHR_state, integ_min, integ_max);
        }

        // Rate limit PD + FF throttle
        // Calculate the throttle increment from the specified slew time
        int8_t throttle_slewrate = aparm.throttle_slewrate;
        if (_landing.is_on_approach()) {
            const int8_t land_slewrate = _landing.get_throttle_slewrate();
            if (land_slewrate > 0) {
                throttle_slewrate = land_slewrate;
            }
        }

        if (throttle_slewrate != 0) {
            float thrRateIncr = _DT * (_THRmaxf - THRminf_clipped_to_zero) * throttle_slewrate * 0.01f;

            _throttle_dem = constrain_float(_throttle_dem,
                                            _last_throttle_dem - thrRateIncr,
                                            _last_throttle_dem + thrRateIncr);
            _last_throttle_dem = _throttle_dem;
        }

        // Sum the components.
        _throttle_dem = _throttle_dem + _integTHR_state;
    }

    // Constrain throttle demand
    _throttle_dem = constrain_float(_throttle_dem, _THRminf, _THRmaxf);
}

float AP_TECS::_get_i_gain(void)
{
    float i_gain = _integGain;
    if (_flight_stage == AP_Vehicle::FixedWing::FLIGHT_TAKEOFF || _flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND) {
        if (!is_zero(_integGain_takeoff)) {
            i_gain = _integGain_takeoff;
        }
    } else if (_flags.is_doing_auto_land) {
        if (!is_zero(_integGain_land)) {
            i_gain = _integGain_land;
        }
    }
    return i_gain;
}

/*
  calculate throttle, non-airspeed case
 */
void AP_TECS::_update_throttle_without_airspeed(int16_t throttle_nudge)
{
    // Calculate throttle demand by interpolating between pitch and throttle limits
    float nomThr;
    //If landing and we don't have an airspeed sensor and we have a non-zero
    //TECS_LAND_THR param then use it
    if (_flags.is_doing_auto_land && _landThrottle >= 0) {
        nomThr = (_landThrottle + throttle_nudge) * 0.01f;
    } else { //not landing or not using TECS_LAND_THR parameter
        nomThr = (aparm.throttle_cruise + throttle_nudge)* 0.01f;
    }

    if (_pitch_dem > 0.0f && _PITCHmaxf > 0.0f)
    {
        _throttle_dem = nomThr + (_THRmaxf - nomThr) * _pitch_dem / _PITCHmaxf;
    }
    else if (_pitch_dem < 0.0f && _PITCHminf < 0.0f)
    {
        _throttle_dem = nomThr + (_THRminf - nomThr) * _pitch_dem / _PITCHminf;
    }
    else
    {
        _throttle_dem = nomThr;
    }

    if (_flags.is_gliding)
    {
        _throttle_dem = 0.0f;
    }

    // Calculate additional throttle for turn drag compensation including throttle nudging
    const Matrix3f &rotMat = _ahrs.get_rotation_body_to_ned();
    // Use the demanded rate of change of total energy as the feed-forward demand, but add
    // additional component which scales with (1/cos(bank angle) - 1) to compensate for induced
    // drag increase during turns.
    float cosPhi = sqrtf((rotMat.a.y*rotMat.a.y) + (rotMat.b.y*rotMat.b.y));
    float STEdot_dem = _rollComp * (1.0f/constrain_float(cosPhi * cosPhi , 0.1f, 1.0f) - 1.0f);
    _throttle_dem = _throttle_dem + STEdot_dem / (_STEdot_max - _STEdot_min) * (_THRmaxf - _THRminf);
}

void AP_TECS::_detect_bad_descent(void)
{
    // Detect a demanded airspeed too high for the aircraft to achieve. This will be
    // evident by the the following conditions:
    // 1) Underspeed protection not active
    // 2) Specific total energy error > 200 (greater than ~20m height error)
    // 3) Specific total energy reducing
    // 4) throttle demand > 90%
    // If these four conditions exist simultaneously, then the protection
    // mode will be activated.
    // Once active, the following condition are required to stay in the mode
    // 1) Underspeed protection not active
    // 2) Specific total energy error > 0
    // This mode will produce an undulating speed and height response as it cuts in and out but will prevent the aircraft from descending into the ground if an unachievable speed demand is set
    float STEdot = _SPEdot + _SKEdot;
    if (((!_flags.underspeed && (_STE_error > 200.0f) && (STEdot < 0.0f) && (_throttle_dem >= _THRmaxf * 0.9f)) || (_flags.badDescent && !_flags.underspeed && (_STE_error > 0.0f))) && !_flags.is_gliding)
    {
        _flags.badDescent = true;
    }
    else
    {
        _flags.badDescent = false;
    }
}

void AP_TECS::_update_pitch(void)
{
    float max_sink_rate = _maxSinkRate;
    float max_climb_rate = _maxClimbRate;
    if (_maxSinkRate_approach > 0 && _flags.is_doing_auto_land) {
        // special sink rate for approach to accommodate steep slopes and reverse thrust.
        // A special check must be done to see if we're LANDing on approach but also if
        // we're in that tiny window just starting NAV_LAND but still in NORMAL mode. If
        // we have a steep slope with a short approach we'll want to allow acquiring the
        // glide slope right away.
        max_sink_rate = _maxSinkRate_approach;
    } else if (_flags.is_gliding) {
        // Special case where it is important that the vertical speed limits
        // are matched to the pitch limits becasue pitch angle is the only method
        // of controlling airspeed when gliding.
        max_sink_rate  = - _TAS_dem * sinf(_PITCHminf - radians(_trim_aoa));
        max_climb_rate = _TAS_dem * sinf(_PITCHmaxf - radians(_trim_aoa));
    }

    // Calculate Speed/Height Control Weighting
    // This is used to determine how the pitch control prioritises speed and height control
    // A weighting of 1 provides equal priority (this is the normal mode of operation)
    // A SKE_weighting of 0 provides 100% priority to height control. This is used when no airspeed measurement is available
    // A SKE_weighting of 2 provides 100% priority to speed control. This is used when an underspeed condition is detected. In this instance, if airspeed
    // rises above the demanded value, the pitch angle will be increased by the TECS controller.
    _SKE_weighting = constrain_float(_spdWeight, 0.0f, 2.0f);
    if (!(_ahrs.airspeed_sensor_enabled() || _use_synthetic_airspeed)) {
        _SKE_weighting = 0.0f;
    } else if (_flight_stage == AP_Vehicle::FixedWing::FLIGHT_VTOL) {
        // if we are in VTOL mode then control pitch without regard to
        // speed. Speed is also taken care of independently of
        // height. This is needed as the usual relationship of speed
        // and height is broken by the VTOL motors
        _SKE_weighting = 0.0f;
    } else if ( _flags.underspeed || _flight_stage == AP_Vehicle::FixedWing::FLIGHT_TAKEOFF || _flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND || _flags.is_gliding) {
        _SKE_weighting = 2.0f;
    } else if (_flags.is_doing_auto_land) {
        if (_spdWeightLand < 0) {
            // use sliding scale from normal weight down to zero at landing
            float scaled_weight = _spdWeight * (1.0f - constrain_float(_path_proportion,0,1));
            _SKE_weighting = constrain_float(scaled_weight, 0.0f, 2.0f);
        } else {
            _SKE_weighting = constrain_float(_spdWeightLand, 0.0f, 2.0f);
        }
    }

    float SPE_weighting = 2.0f - _SKE_weighting;

    // either weight can fade to 0, but don't go above 1 to prevent instability if tuned at a speed weight of 1 and wieghting is varied to end points in flight.
    SPE_weighting = MIN(SPE_weighting, 1.0f);
    _SKE_weighting = MIN(_SKE_weighting, 1.0f);

    // Calculate demanded specific energy balance and error
    float SEB_dem   = _SPE_dem * SPE_weighting - _SKE_dem * _SKE_weighting;
    float SEB_error = SEB_dem - (_SPE_est * SPE_weighting - _SKE_est * _SKE_weighting);

    // track demanded height using the specified time constant
    const float SEBdot_dem_ff1 = (_flags.is_gliding) ? 0.0f : _hgt_rate_dem * GRAVITY_MSS;
    float SEBdot_dem = constrain_float(SEBdot_dem_ff1 + SEB_error / timeConstant(), -max_sink_rate * GRAVITY_MSS, max_climb_rate * GRAVITY_MSS);

    // rate of change of potential energy is required by total energy controller
    _SPEdot_dem = (_SPE_dem * SPE_weighting - _SPE_est) * SPE_weighting / timeConstant();

    // calculate specific energy balance rate error
    float SEBdot_error = SEBdot_dem - (_SPEdot * SPE_weighting - _SKEdot * _SKE_weighting);

    logging.SKE_error = _SKE_dem - _SKE_est;
    logging.SPE_error = _SPE_dem - _SPE_est;
    
    // sum predicted plus damping correction
    // integral correction is added later
    // During flare a different damping gain is used
    float pitch_damp = _ptchDamp;
    if (_landing.is_flaring()) {
        pitch_damp = _landDamp;
    } else if (!is_zero(_land_pitch_damp) && _flags.is_doing_auto_land) {
        pitch_damp = _land_pitch_damp;
    }
    const float SEBdot_dem_ff2 = (_flags.is_gliding) ? - _SKEdot_dem * _SKE_weighting : 0.0f;
    float SEBdot_dem_total = SEBdot_dem_ff2 + SEBdot_dem + SEBdot_error * pitch_damp;

    // inverse of gain from SEB to pitch angle
    float gainInv = (_TAS_state * GRAVITY_MSS);

    // During climbout/takeoff, bias the demanded pitch angle so that zero speed error produces a pitch angle
    // demand equal to the minimum value (which is )set by the mission plan during this mode). Otherwise the
    // integrator has to catch up before the nose can be raised to reduce speed during climbout.
    if (_flight_stage == AP_Vehicle::FixedWing::FLIGHT_TAKEOFF || _flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND) {
        SEBdot_dem_total += _PITCHminf * gainInv;
    }

    // Calculate integrator state, constraining input if pitch limits are exceeded
    // don't allow the integrator to rise by more than 10% of its full
    // range in one step. This prevents single value glitches from
    // causing massive integrator changes. See Issue#4066
    float integSEB_range = gainInv * (_PITCHmaxf -_PITCHminf);
    float integSEB_delta = constrain_float(SEBdot_error * _get_i_gain() * _DT, -integSEB_range*0.1f, integSEB_range*0.1f);

    // predict what pitch will be with uncontrained integration
    _pitch_dem_unc = (SEBdot_dem_total + _integSEB_state + integSEB_delta) / gainInv;

    // integrate SEB rate error and apply integrator state limits
    const bool inhibit_integrator = ((_pitch_dem_unc > _PITCHmaxf || _pitch_rate_clip_state == Saturation::HIGH) && integSEB_delta > 0.0f) ||
                                    ((_pitch_dem_unc < _PITCHminf || _pitch_rate_clip_state == Saturation::LOW ) && integSEB_delta < 0.0f);
    if (!inhibit_integrator) {
        _integSEB_state += integSEB_delta;
    } else if (is_positive(integSEB_delta * _hgt_rate_err_integ)) {
        // fade out integrator if saturating
        _integSEB_state *= (1.0f - _DT / timeConstant());
    }
    _integSEB_state = constrain_float(_integSEB_state, -0.5f * integSEB_range, 0.5f * integSEB_range);

    logging.SEB_delta = integSEB_delta;
    
    // Calculate pitch demand from specific energy balance signals
    _pitch_dem_unc = (SEBdot_dem_total + _integSEB_state) / gainInv;

    // Add a feedforward term from demanded airspeed to pitch
    if (_flags.is_gliding) {
        _pitch_dem_unc += (_EAS_dem_rlim - _pitch_ff_v0) * _pitch_ff_k;
    }

    // Add trim angle of attack
    _pitch_dem_unc += radians(_trim_aoa);

    // Constrain pitch demand
    _pitch_dem = constrain_float(_pitch_dem_unc, _PITCHminf, _PITCHmaxf);

    // Rate limit the pitch demand to comply with specified vertical
    // acceleration limit
    float ptchRateIncr = _DT * _vertAccLim / _TAS_state;

    if ((_pitch_dem - _last_pitch_dem) > ptchRateIncr)
    {
        _pitch_rate_clip_state = Saturation::HIGH;
        _pitch_dem = _last_pitch_dem + ptchRateIncr;
    }
    else if ((_pitch_dem - _last_pitch_dem) < -ptchRateIncr)
    {
        _pitch_dem = _last_pitch_dem - ptchRateIncr;
        _pitch_rate_clip_state = Saturation::LOW;
    } else {
        _pitch_rate_clip_state = Saturation::NONE;
    }

    _last_pitch_dem = _pitch_dem;
    AP::logger().Write("TEC3","TimeUS,PEW,EBD,EBE,EBDD,EBDE,EBDDT,Irng,I","Qffffffff",
                    AP_HAL::micros64(),
                    (double)SPE_weighting,      // PEW
                    (double)SEB_dem,            // EBD
                    (double)SEB_error,          // EBE
                    (double)SEBdot_dem,         // EBDD
                    (double)SEBdot_error,       // EBDE
                    (double)SEBdot_dem_total,   // EBDDT
                    (double)integSEB_range,     // Irng
                    (double)_integSEB_state);   // I
}

void AP_TECS::_initialise_states(int32_t ptchMinCO_cd, float hgt_afe)
{
    // Initialise states and variables if DT > 0.2 second or in climbout
    if (_DT > 0.2f || _need_reset)
    {
        _SKE_weighting        = 1.0f;
        _integTHR_state       = 0.0f;
        _integSEB_state       = 0.0f;
        _last_throttle_dem    = aparm.throttle_cruise * 0.01f;
        _last_pitch_dem       = _ahrs.pitch;
        _hgt_afe              = hgt_afe;
        _hgt_dem_in_prev      = hgt_afe;
        _hgt_dem_lpf          = hgt_afe;
        _hgt_dem_rate_ltd     = hgt_afe;
        _update_airspeed_conversions();
        _reset_airspeed_states();
        _DT                   = 0.1f; // when first starting TECS, use the most likely time constant
        _lag_comp_hgt_offset  = 0.0f;
        _post_TO_hgt_offset   = 0.0f;

        _flags.underspeed            = false;
        _flags.badDescent            = false;
        _flags.reached_speed_takeoff = false;
        _need_reset                  = false;

        // misc variables used for alternative precision landing pitch control
        _hgt_rate_err_integ       = 0.0f;
        _hgt_at_start_of_flare    = 0.0f;
        _hgt_rate_at_flare_entry  = 0.0f;
        _hgt_afe                  = 0.0f;
        _pitch_min_at_flare_entry = 0.0f;
    }
    else if (_flight_stage == AP_Vehicle::FixedWing::FLIGHT_TAKEOFF || _flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND)
    {
        _PITCHminf            = 0.000174533f * ptchMinCO_cd;
        _hgt_afe              = hgt_afe;
        _hgt_dem_lpf          = hgt_afe;
        _hgt_dem_rate_ltd = hgt_afe;
        _reset_airspeed_states();
        _post_TO_hgt_offset   = _climb_rate * _hgt_dem_tconst;
        _flags.underspeed     = false;
        _flags.badDescent     = false;
    }
    
    if (_flight_stage != AP_Vehicle::FixedWing::FLIGHT_TAKEOFF && _flight_stage != AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND) {
        // reset takeoff speed flag when not in takeoff
        _flags.reached_speed_takeoff = false;        
    }
}

void AP_TECS::_update_STE_rate_lim(void)
{
    // Calculate Specific Total Energy Rate Limits
    // This is a trivial calculation at the moment but will get bigger once we start adding altitude effects
    _STEdot_max = _maxClimbRate * GRAVITY_MSS;
    _STEdot_min = - _minSinkRate * GRAVITY_MSS;
}


void AP_TECS::update_pitch_throttle(int32_t hgt_dem_cm,
                                    int32_t EAS_dem_cm,
                                    enum AP_Vehicle::FixedWing::FlightStage flight_stage,
                                    float distance_beyond_land_wp,
                                    int32_t ptchMinCO_cd,
                                    int16_t throttle_nudge,
                                    float hgt_afe,
                                    float load_factor)
{
    // Calculate time in seconds since last update
    uint64_t now = AP_HAL::micros64();
    _DT = (now - _update_pitch_throttle_last_usec) * 1.0e-6f;
    _DT = MAX(_DT, 0.001f);
    _update_pitch_throttle_last_usec = now;

    _flags.is_gliding = ((_options & OPTION_GLIDER_ONLY) != 0) || _flags.gliding_requested || _flags.propulsion_failed || aparm.throttle_max==0;
    _flags.is_doing_auto_land = (flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND);
    _distance_beyond_land_wp = distance_beyond_land_wp;
    _flight_stage = flight_stage;

    // Convert inputs
    _hgt_dem_in = hgt_dem_cm * 0.01f;
    _EAS_dem = EAS_dem_cm * 0.01f;
    _hgt_afe = hgt_afe;

    // Update the speed estimate using a 2nd order complementary filter
    _update_speed(load_factor);

    if (aparm.takeoff_throttle_max != 0 &&
            (_flight_stage == AP_Vehicle::FixedWing::FLIGHT_TAKEOFF || _flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND)) {
        _THRmaxf  = aparm.takeoff_throttle_max * 0.01f;
    } else {
        _THRmaxf  = aparm.throttle_max * 0.01f;
    }
    _THRminf  = aparm.throttle_min * 0.01f;

    // min of 1% throttle range to prevent a numerical error
    _THRmaxf = MAX(_THRmaxf, _THRminf+0.01);

    // work out the maximum and minimum pitch
    // if TECS_PITCH_{MAX,MIN} isn't set then use
    // LIM_PITCH_{MAX,MIN}. Don't allow TECS_PITCH_{MAX,MIN} to be
    // larger than LIM_PITCH_{MAX,MIN}
    if (_pitch_max == 0) {
        _PITCHmaxf = aparm.pitch_limit_max_cd * 0.01f;
    } else {
        _PITCHmaxf = MIN(_pitch_max, aparm.pitch_limit_max_cd * 0.01f);
    }

    if (_pitch_min >= 0) {
        _PITCHminf = aparm.pitch_limit_min_cd * 0.01f;
    } else {
        _PITCHminf = MAX(_pitch_min, aparm.pitch_limit_min_cd * 0.01f);
    }

    // apply temporary pitch limit and clear
    if (_pitch_max_limit < 90) {
        _PITCHmaxf = constrain_float(_PITCHmaxf, -90, _pitch_max_limit);
        _PITCHminf = constrain_float(_PITCHminf, -_pitch_max_limit, _PITCHmaxf);
        _pitch_max_limit = 90;
    }

    if (!_landing.is_on_approach()) {
        // reset land pitch min when not landing
        _land_pitch_min = _PITCHminf;
    }
    
    // calculate the expected pitch angle from the demanded climb rate and airspeed fo ruse during approach and flare
    if (_landing.is_flaring()) {
        // smoothly move the min pitch to the required minimum at touchdown
        float p; // 0 at start of flare, 1 at finish
        if (_flare_counter == 0) {
            p = 0.0f;
        } else if (_hgt_at_start_of_flare > _flare_holdoff_hgt) {
            p = constrain_float((_hgt_at_start_of_flare - _hgt_afe) / _hgt_at_start_of_flare, 0.0f, 1.0f);
        } else {
            p = 1.0f;
        }
        const float pitch_limit_deg = (1.0f - p) * _pitch_min_at_flare_entry + p * 0.01f * _landing.get_pitch_cd();

        // in flare use min pitch from LAND_PITCH_CD
        _PITCHminf = MAX(_PITCHminf, pitch_limit_deg);

        // and use max pitch from TECS_LAND_PMAX
        if (_land_pitch_max != 0) {
            // note that this allows a flare pitch outside the normal TECS auto limits
            _PITCHmaxf = _land_pitch_max;
        }

        // and allow zero throttle
        _THRminf = 0;
    } else if (_landing.is_on_approach()) {
        _PITCHminf = MAX(_PITCHminf, 0.01f * aparm.pitch_limit_min_cd);
        _pitch_min_at_flare_entry = _PITCHminf;
    }

    if (_landing.is_on_approach()) {
        // don't allow the lower bound of pitch to decrease, nor allow
        // it to increase rapidly. This prevents oscillation of pitch
        // demand while in landing approach based on rapidly changing
        // time to flare estimate
        if (_land_pitch_min <= -90) {
            _land_pitch_min = _PITCHminf;
        }
        const float flare_pitch_range = 20;
        const float delta_per_loop = (flare_pitch_range/_landTimeConst) * _DT;
        _PITCHminf = MIN(_PITCHminf, _land_pitch_min+delta_per_loop);
        _land_pitch_min = MAX(_land_pitch_min, _PITCHminf);
        _PITCHminf = MAX(_land_pitch_min, _PITCHminf);
    }

    if (_landing.is_flaring()) {
        // ensure we don't violate the limits for flare pitch
        if (_land_pitch_max != 0) {
            _PITCHmaxf = MIN(_land_pitch_max, _PITCHmaxf);
        }
    }

    if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_TAKEOFF || flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND) {
        if (!_flags.reached_speed_takeoff && _TAS_state >= _TAS_dem) {
            // we have reached our target speed in takeoff, allow for
            // normal throttle control
            _flags.reached_speed_takeoff = true;
        }
    }
    
    // convert to radians
    _PITCHmaxf = radians(_PITCHmaxf);
    _PITCHminf = radians(_PITCHminf);

    // don't allow max pitch to go below min pitch
    _PITCHmaxf = MAX(_PITCHmaxf, _PITCHminf);

    // initialise selected states and variables if DT > 1 second or in climbout
    _initialise_states(ptchMinCO_cd, hgt_afe);

    // Calculate Specific Total Energy Rate Limits
    _update_STE_rate_lim();

    // Calculate the speed demand
    _update_speed_demand();

    // Calculate the height demand
    _update_height_demand();

    // Detect underspeed condition
    _detect_underspeed();

    // Calculate specific energy quantitiues
    _update_energies();

    // Calculate pitch demand
    _update_pitch();

    // Calculate throttle demand - use simple pitch to throttle if no
    // airspeed sensor.
    // Note that caller can demand the use of
    // synthetic airspeed for one loop if needed. This is required
    // during QuadPlane transition when pitch is constrained
    if (_ahrs.airspeed_sensor_enabled() || _use_synthetic_airspeed || _use_synthetic_airspeed_once) {
        _update_throttle_with_airspeed();
        _use_synthetic_airspeed_once = false;
    } else {
        _update_throttle_without_airspeed(throttle_nudge);
    }

    // Detect bad descent due to demanded airspeed being too high
    _detect_bad_descent();

    // log to AP_Logger
    // @LoggerMessage: TECS
    // @Vehicles: Plane
    // @Description: Information about the Total Energy Control System
    // @URL: http://ardupilot.org/plane/docs/tecs-total-energy-control-system-for-speed-height-tuning-guide.html
    // @Field: TimeUS: Time since system startup
    // @Field: h: height estimate (UP) currently in use by TECS
    // @Field: dh: current climb rate ("delta-height")
    // @Field: hdem: height TECS is currently trying to achieve
    // @Field: dhdem: climb rate TECS is currently trying to achieve
    // @Field: spdem: True AirSpeed TECS is currently trying to achieve
    // @Field: sp: current estimated True AirSpeed
    // @Field: dsp: x-axis acceleration estimate ("delta-speed")
    // @Field: ith: throttle integrator value
    // @Field: iph: Specific Energy Balance integrator value
    // @Field: th: throttle output
    // @Field: ph: pitch output
    // @Field: dspdem: demanded acceleration output ("delta-speed demand")
    // @Field: w: current TECS prioritization of height vs speed (0==100% height,2==100% speed, 1==50%height+50%speed
    // @Field: f: flags
    // @FieldBits: f: Underspeed,UnachievableDescent,AutoLanding,ReachedTakeoffSpd
    AP::logger().Write(
        "TECS",
        "TimeUS,h,dh,hin,hdem,dhdem,spdem,sp,dsp,ith,iph,th,ph,dspdem,w,f",
        "smnmmnnnn----o--",
        "F00000000----0--",
        "QffffffffffffffB",
        now,
        (double)_height,
        (double)_climb_rate,
        (double)_hgt_dem_in,
        (double)_hgt_dem,
        (double)_hgt_rate_dem,
        (double)_TAS_dem,
        (double)_TAS_state,
        (double)_vel_dot_hpf_out,
        (double)_integTHR_state,
        (double)_integSEB_state,
        (double)_throttle_dem,
        (double)_pitch_dem,
        (double)_TAS_rate_dem,
        (double)_SKE_weighting,
        _flags_byte);
    // @LoggerMessage: TEC2
    // @Vehicles: Plane
    // @Description: Additional Information about the Total Energy Control System
    // @URL: http://ardupilot.org/plane/docs/tecs-total-energy-control-system-for-speed-height-tuning-guide.html
    // @Field: TimeUS: Time since system startup
    // @Field: pmax: maximum allowed pitch from parameter
    // @Field: pmin: minimum allowed pitch from parameter
    // @Field: KErr: difference between estimated kinetic energy and desired kinetic energy
    // @Field: PErr: difference between estimated potential energy and desired potential energy
    // @Field: EDelta: current error in speed/balance weighting
    // @Field: LF: aerodynamic load factor as received
    // @Field: EASdem: demanded equivalent airspeed as received
    AP::logger().Write("TEC2", "TimeUS,pmax,pmin,KErr,PErr,EDelta,LF,EASdem",
                       "s-------",
                       "F-------",
                       "Qfffffff",
                       now,
                       (double)degrees(_PITCHmaxf),
                       (double)degrees(_PITCHminf),
                       (double)logging.SKE_error,
                       (double)logging.SPE_error,
                       (double)logging.SEB_delta,
                       (double)load_factor,
                       (double)_EAS_dem);
}

void AP_TECS::_update_airspeed_conversions(void)
{
    // limit value to range in AP_Baro atmosphere tables
    _EAS2TAS = constrain_float(_ahrs.get_EAS2TAS(), sqrtf(1.22506f/1.86000f), sqrtf(1.22506f/0.00170f));
    _TAS2EAS = 1.0f / _EAS2TAS;
}

void AP_TECS::_reset_airspeed_states(void)
{
        if (!_ahrs.airspeed_estimate(_EAS)) {
            // If no airspeed available use average of min and max
            _EAS = 0.5f * (aparm.airspeed_min.get() + (float)aparm.airspeed_max.get());
        }
        _EAS_dem_rlim = _EAS_dem_lpf_1 = _EAS_dem_lpf_2 = _EAS_dem = _EAS;
        _integDTAS_state = 0.0f;
        _TAS_state = _EAS * _EAS2TAS;
        _TAS_dem = _TAS_state;
}
