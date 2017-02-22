/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *   AP_Landing_Deepstall.cpp - Landing logic handler for ArduPlane for deepstall landings
 */

#include "AP_Landing.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>

// table of user settable parameters for deepstall
const AP_Param::GroupInfo AP_Landing::deepstall_var_info[] = {

    // @Param: DS_V_FWD
    // @DisplayName: Deepstall forward velocity
    // @Description: The forward velocity of the aircraft while stalled
    // @Range: 0 20
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("V_FWD", 1, AP_Landing, deepstall.forward_speed, 1),

    // @Param: DS_SLOPE_A
    // @DisplayName: Deepstall slope a
    // @Description: The a component of distance = a*wind + b
    // @User: Advanced
    AP_GROUPINFO("SLOPE_A", 2, AP_Landing, deepstall.slope_a, 1),

    // @Param: DS_SLOPE_B
    // @DisplayName: Deepstall slope b
    // @Description: The a component of distance = a*wind + b
    // @User: Advanced
    AP_GROUPINFO("SLOPE_B", 3, AP_Landing, deepstall.slope_b, 1),

    // @Param: DS_APP_EXT
    // @DisplayName: Deepstall approach extension
    // @Description: The forward velocity of the aircraft while stalled
    // @Range: 10 200
    // @Units: meters
    // @User: Advanced
    AP_GROUPINFO("APP_EXT", 4, AP_Landing, deepstall.approach_extension, 50),

    // @Param: DS_V_DWN
    // @DisplayName: Deepstall veloicty down
    // @Description: The downward velocity of the aircraft while stalled
    // @Range: 0 20
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("V_DWN", 5, AP_Landing, deepstall.down_speed, 2),

    // @Param: DS_SLEW_SPD
    // @DisplayName: Deepstall slew speed
    // @Description: The speed at which the elevator slews to deepstall
    // @Range: 0 2
    // @Units: seconds
    // @User: Advanced
    AP_GROUPINFO("SLEW_SPD", 6, AP_Landing, deepstall.slew_speed, 0.5),

    // @Param: DS_ELEV_PWM
    // @DisplayName: Deepstall elevator PWM
    // @Description: The PWM value for the elevator at full deflection in deepstall
    // @Range: 900 2100
    // @Units: PWM
    // @User: Advanced
    AP_GROUPINFO("ELEV_PWM", 7, AP_Landing, deepstall.elevator_pwm, 1500),

    // @Param: DS_ARSP_MAX
    // @DisplayName: Deepstall enabled airspeed
    // @Description: The maximum aispeed where the deepstall steering controller is allowed to have control
    // @Range: 5 20
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("ARSP_MAX", 8, AP_Landing, deepstall.handoff_airspeed, 15.0),

    // @Param: DS_ARSP_MIN
    // @DisplayName: Deepstall minimum derating airspeed
    // @Description: Deepstall lowest airspeed where the deepstall controller isn't allowed full control
    // @Range: 5 20
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("ARSP_MIN", 9, AP_Landing, deepstall.handoff_lower_limit_airspeed, 10.0),

    // @Param: DS_L1
    // @DisplayName: Deepstall L1 period
    // @Description: Deepstall L1 navigational controller period
    // @Range: 5 50
    // @Units: meters
    // @User: Advanced
    AP_GROUPINFO("L1", 10, AP_Landing, deepstall.L1_period, 30.0),

    // @Param: DS_L1_I
    // @DisplayName: Deepstall L1 I gain
    // @Description: Deepstall L1 integratior gain
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("L1_I", 11, AP_Landing, deepstall.L1_i, 0),

    // @Param: DS_YAW_LIM
    // @DisplayName: Deepstall yaw rate limit
    // @Description: The yaw rate limit while navigating in deepstall
    // @Range: 0 90
    // @Units degrees per second
    // @User: Advanced
    AP_GROUPINFO("YAW_LIM", 12, AP_Landing, deepstall.yaw_rate_limit, 10),

    // @Param: DS_L1_TCON
    // @DisplayName: Deepstall L1 time constant
    // @Description: Time constant for deepstall L1 control
    // @Range: 0 1
    // @Units seconds
    // @User: Advanced
    AP_GROUPINFO("L1_TCON", 13, AP_Landing, deepstall.time_constant, 0.4),

    // @Group: DS_
    // @Path: ../PID/PID.cpp
    AP_SUBGROUPINFO(deepstall.ds_PID, "", 13, AP_Landing, PID),

    AP_GROUPEND
};


// if DEBUG_PRINTS is defined statustexts will be sent to the GCS for debug purposes
//#define DEBUG_PRINTS

void AP_Landing::type_deepstall_do_land(const AP_Mission::Mission_Command& cmd, const float relative_altitude)
{
    deepstall.stage = DEEPSTALL_STAGE_FLY_TO_LANDING;
    deepstall.ds_PID.reset_I();

    // load the landing point in, the rest of path building is deferred for a better wind estimate
    memcpy(&deepstall.landing_point, &cmd.content.location, sizeof(Location));
}

// currently identical to the slope aborts
void AP_Landing::type_deepstall_verify_abort_landing(const Location &prev_WP_loc, Location &next_WP_loc, bool &throttle_suppressed)
{
    // when aborting a landing, mimic the verify_takeoff with steering hold. Once
    // the altitude has been reached, restart the landing sequence
    throttle_suppressed = false;
    nav_controller->update_heading_hold(get_bearing_cd(prev_WP_loc, next_WP_loc));
}


/*
  update navigation for landing
 */
bool AP_Landing::type_deepstall_verify_land(const Location &prev_WP_loc, Location &next_WP_loc, const Location &current_loc,
        const float height, const float sink_rate, const float wp_proportion, const uint32_t last_flying_ms,
        const bool is_armed, const bool is_flying, const bool rangefinder_state_in_range)
{
    switch (deepstall.stage) {
    case DEEPSTALL_STAGE_FLY_TO_LANDING:
        if (get_distance(current_loc, deepstall.landing_point) > 2 * aparm.loiter_radius) {
            nav_controller->update_waypoint(current_loc, deepstall.landing_point);
            return false;
        }
        deepstall.stage = DEEPSTALL_STAGE_ESTIMATE_WIND;
        deepstall.loiter_sum_cd = 0; // reset the loiter counter
        // no break
    case DEEPSTALL_STAGE_ESTIMATE_WIND:
        {
        if (!nav_controller->reached_loiter_target() || (fabsf(height) > DEEPSTALL_LOITER_ALT_TOLERANCE)) {
            // wait until the altitude is correct before considering a breakout
            nav_controller->update_loiter(deepstall.landing_point, aparm.loiter_radius, 1);
            return false;
        }
        // only count loiter progress when within the target altitude
        int32_t target_bearing = nav_controller->target_bearing_cd();
        int32_t delta = wrap_180_cd(target_bearing - deepstall.last_target_bearing);
        if (delta > 0) { // only accumulate turns in the correct direction
            deepstall.loiter_sum_cd += delta;
        }
        deepstall.last_target_bearing = target_bearing;
        if (deepstall.loiter_sum_cd < 36000) {
            // wait until we've done at least one complete loiter at the correct altitude
            nav_controller->update_loiter(deepstall.landing_point, aparm.loiter_radius, 1);
            return false;
        }
        deepstall.stage = DEEPSTALL_STAGE_WAIT_FOR_BREAKOUT;
        //compute optimal path for landing
        type_deepstall_build_approach_path();
        // no break
        }
    case DEEPSTALL_STAGE_WAIT_FOR_BREAKOUT:
        if (!type_deepstall_verify_breakout(current_loc, deepstall.arc_entry, height)) {
            nav_controller->update_loiter(deepstall.landing_point, aparm.loiter_radius, 1);
            return false;
        }
        deepstall.stage = DEEPSTALL_STAGE_FLY_TO_ARC;
        memcpy(&deepstall.breakout_location, &current_loc, sizeof(Location));
        // no break
    case DEEPSTALL_STAGE_FLY_TO_ARC:
        if (get_distance(current_loc, deepstall.arc_entry) > 2 * aparm.loiter_radius) {
            nav_controller->update_waypoint(deepstall.breakout_location, deepstall.arc_entry);
            return false;
        }
        deepstall.stage = DEEPSTALL_STAGE_ARC;
        // no break
    case DEEPSTALL_STAGE_ARC:
        {
        Vector2f groundspeed = ahrs.groundspeed_vector();
        if (!nav_controller->reached_loiter_target() ||
            (fabsf(wrap_180(deepstall.target_heading_deg -
                            degrees(atan2f(-groundspeed.y, -groundspeed.x) + M_PI))) >= 10.0f)) {
            nav_controller->update_loiter(deepstall.arc, aparm.loiter_radius, 1);
            return false;
        }
        deepstall.stage = DEEPSTALL_STAGE_APPROACH;
        }
        // no break
    case DEEPSTALL_STAGE_APPROACH:
        {
        Location entry_point;
        nav_controller->update_waypoint(deepstall.arc_exit, deepstall.extended_approach);

        float relative_alt_D;
        ahrs.get_relative_position_D_home(relative_alt_D);

        const float travel_distance = type_deepstall_predict_travel_distance(ahrs.wind_estimate(), -relative_alt_D);

        memcpy(&entry_point, &deepstall.landing_point, sizeof(Location));
        location_update(entry_point, deepstall.target_heading_deg + 180.0, travel_distance);

        if (!location_passed_point(current_loc, deepstall.arc_exit, entry_point)) {
            if (location_passed_point(current_loc, deepstall.arc_exit, deepstall.extended_approach)) {
                // this should never happen, but prevent against an indefinite fly away
                deepstall.stage = DEEPSTALL_STAGE_FLY_TO_LANDING;
            }
            return false;
        }
        deepstall.stage = DEEPSTALL_STAGE_LAND;
        deepstall.stall_entry_time = AP_HAL::millis();

        const SRV_Channel* elevator = SRV_Channels::get_channel_for(SRV_Channel::k_elevator);
        if (elevator != nullptr) {
            // take the last used elevator angle as the starting deflection
            // don't worry about bailing here if the elevator channel can't be found
            // that will be handled within override_servos
            deepstall.initial_elevator_pwm = elevator->get_output_pwm();
        }
        deepstall.L1_xtrack_i = 0; // reset the integrators
        }
        // no break
    case DEEPSTALL_STAGE_LAND:
        // while in deepstall the only thing verify needs to keep the extended approach point sufficently far away
        nav_controller->update_waypoint(current_loc, deepstall.extended_approach);
        return false;
    default:
        return true;
    }
}

bool AP_Landing::type_deepstall_override_servos(void)
{
    if (!(deepstall.stage == DEEPSTALL_STAGE_LAND)) {
        return false;
    }

    SRV_Channel* elevator = SRV_Channels::get_channel_for(SRV_Channel::k_elevator);
    SRV_Channel* aileron = SRV_Channels::get_channel_for(SRV_Channel::k_aileron);
    SRV_Channel* rudder = SRV_Channels::get_channel_for(SRV_Channel::k_rudder);

    if (elevator == nullptr || aileron == nullptr || rudder == nullptr) {
        // deepstalls are impossible without these channels, abort the process
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,
                                         "Deepstall: Unable to find aileron, rudder, and elevator channels");
        type_deepstall_request_go_around();
        return false;
    }

    // calculate the progress on slewing the elevator
    float slew_progress = 1.0f;
    if (deepstall.slew_speed > 0) {
        slew_progress  = (AP_HAL::millis() - deepstall.stall_entry_time) / (100.0f * deepstall.slew_speed);
        slew_progress = constrain_float (slew_progress, 0.0f, 1.0f);
    }

    // mix the elevator to the correct value
    elevator->set_output_pwm(linear_interpolate(deepstall.initial_elevator_pwm, deepstall.elevator_pwm,
                             slew_progress, 0.0f, 1.0f));

    // use the current airspeed to dictate the travel limits
    float airspeed;
    ahrs.airspeed_estimate(&airspeed);

    // only allow the deepstall steering controller to run below the handoff airspeed
    if (slew_progress >= 1.0f || airspeed <= deepstall.handoff_airspeed) {
        // run the steering conntroller
        float pid = type_deepstall_update_steering();


        float travel_limit = constrain_float((deepstall.handoff_airspeed - airspeed) /
                                             (deepstall.handoff_airspeed - deepstall.handoff_lower_limit_airspeed) *
                                             0.5f + 0.5f,
                                             0.5f, 1.0f);

        float output = constrain_float(pid, -travel_limit, travel_limit);
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, output*4500);
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron_with_input, output*4500);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, output*4500);

    } else {
        // allow the normal servo control of the channel
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron_with_input,
                                        SRV_Channels::get_output_scaled(SRV_Channel::k_aileron));
    }

    // hand off rudder control to deepstall controlled
    return true;
}

bool AP_Landing::type_deepstall_request_go_around(void)
{
    flags.commanded_go_around = true;
    return true;
}

bool AP_Landing::type_deepstall_is_throttle_suppressed(void) const
{
    return deepstall.stage == DEEPSTALL_STAGE_LAND;
}

bool AP_Landing::type_deepstall_get_target_altitude_location(Location &location)
{
    memcpy(&location, &deepstall.landing_point, sizeof(Location));
    return true;
}

int32_t AP_Landing::type_deepstall_get_target_airspeed_cm(void) const
{
    if (deepstall.stage == DEEPSTALL_STAGE_APPROACH ||
        deepstall.stage == DEEPSTALL_STAGE_LAND) {
        return pre_flare_airspeed * 100;
    } else {
        return aparm.airspeed_cruise_cm;
    }
}

const DataFlash_Class::PID_Info& AP_Landing::type_deepstall_get_pid_info(void) const
{
    return deepstall.ds_PID.get_pid_info();
}

void AP_Landing::type_deepstall_build_approach_path(void)
{
    Vector3f wind = ahrs.wind_estimate();
    // TODO: Support a user defined approach heading
    deepstall.target_heading_deg = (degrees(atan2f(-wind.y, -wind.x)));

    memcpy(&deepstall.extended_approach, &deepstall.landing_point, sizeof(Location));
    memcpy(&deepstall.arc_exit, &deepstall.landing_point, sizeof(Location));

    //extend the approach point to 1km away so that there is always a navigational target
    location_update(deepstall.extended_approach, deepstall.target_heading_deg, 1000.0);

    float expected_travel_distance = type_deepstall_predict_travel_distance(wind, deepstall.landing_point.alt / 100);
    float approach_extension = expected_travel_distance + deepstall.approach_extension;
    // an approach extension of 0 can result in a divide by 0
    if (fabsf(approach_extension) < 1.0f) {
        approach_extension = 1.0f;
    }

    location_update(deepstall.arc_exit, deepstall.target_heading_deg + 180, approach_extension);
    memcpy(&deepstall.arc, &deepstall.arc_exit, sizeof(Location));
    memcpy(&deepstall.arc_entry, &deepstall.arc_exit, sizeof(Location));

    // TODO: Support loitering on either side of the approach path
    location_update(deepstall.arc, deepstall.target_heading_deg + 90.0, aparm.loiter_radius);
    location_update(deepstall.arc_entry, deepstall.target_heading_deg + 90.0, aparm.loiter_radius * 2);

#ifdef DEBUG_PRINTS
    // TODO: Send this information via a MAVLink packet
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Arc: %3.8f %3.8f",
                                     (double)(deepstall.arc.lat / 1e7),(double)( deepstall.arc.lng / 1e7));
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Loiter en: %3.8f %3.8f",
                                     (double)(deepstall.arc_entry.lat / 1e7), (double)(deepstall.arc_entry.lng / 1e7));
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Loiter ex: %3.8f %3.8f",
                                     (double)(deepstall.arc_exit.lat / 1e7), (double)(deepstall.arc_exit.lng / 1e7));
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Extended: %3.8f %3.8f",
                                     (double)(deepstall.extended_approach.lat / 1e7), (double)(deepstall.extended_approach.lng / 1e7));
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Extended by: %f (%f)", (double)approach_extension,
                                     (double)expected_travel_distance);
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Target Heading: %3.1f", (double)deepstall.target_heading_deg);
#endif // DEBUG_PRINTS

}

float AP_Landing::type_deepstall_predict_travel_distance(const Vector3f wind, const float height) const
{
    bool reverse = false;

    float course = radians(deepstall.target_heading_deg);

    // a forward speed of 0 will result in a divide by 0
    float forward_speed = MAX(deepstall.forward_speed, 0.1f);

    Vector2f wind_vec(wind.x, wind.y); // work with the 2D component of wind
    float wind_length = MAX(wind_vec.length(), 0.05f); // always assume a slight wind to avoid divide by 0
    Vector2f course_vec(cosf(course), sinf(course));

    float offset = course + atan2f(-wind.y, -wind.x) + M_PI;

    // estimator for how far the aircraft will travel while entering the stall
    float stall_distance = deepstall.slope_a * wind_length * cosf(offset) + deepstall.slope_b;

    float theta = acosf(constrain_float((wind_vec * course_vec) / wind_length, -1.0f, 1.0f));
    if ((course_vec % wind_vec) > 0) {
        reverse = true;
        theta *= -1;
    }

    float cross_component = sinf(theta) * wind_length;
    float estimated_crab_angle = asinf(constrain_float(cross_component / forward_speed, -1.0f, 1.0f));
    if (reverse) {
        estimated_crab_angle *= -1;
    }

    float estimated_forward = cosf(estimated_crab_angle) * forward_speed + cosf(theta) * wind_length;

#ifdef DEBUG_PRINTS
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Predict: %f %f", stall_distance,  estimated_forward * height / deepstall.down_speed + stall_distance);
#endif // DEBUG_PRINTS

    return estimated_forward * height / deepstall.down_speed + stall_distance;
}

bool AP_Landing::type_deepstall_verify_breakout(const Location &current_loc, const Location &target_loc,
                                               const float height_error) const
{
    Vector2f location_delta = location_diff(current_loc, target_loc);
    const float heading_error = degrees(ahrs.groundspeed_vector().angle(location_delta));

    // Check to see if the the plane is heading toward the land waypoint. We use 20 degrees (+/-10 deg)
    // of margin so that the altitude to be within 5 meters of desired

    if (heading_error <= 10.0  && fabsf(height_error) < DEEPSTALL_LOITER_ALT_TOLERANCE) {
            // Want to head in a straight line from _here_ to the next waypoint instead of center of loiter wp
            return true;
    }   
    return false;
}

float AP_Landing::type_deepstall_update_steering()
{
    Location current_loc;
    if (!ahrs.get_position(current_loc)) {
        // panic if no position source is available
        // continue the deepstall. but target just holding the wings held level as deepstall should be a minimal energy
        // configuration on the aircraft, and if a position isn't available aborting would be worse
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "Deepstall: No position available. Attempting to hold level");
        memcpy(&current_loc, &deepstall.landing_point, sizeof(Location));
    }
    uint32_t time = AP_HAL::millis();
    float dt = constrain_float(time - deepstall.last_time, (uint32_t)10UL, (uint32_t)200UL) / 1000.0;
    deepstall.last_time = time;


    Vector2f ab = location_diff(deepstall.arc_exit, deepstall.extended_approach);
    ab.normalize();
    Vector2f a_air = location_diff(deepstall.arc_exit, current_loc);

    float crosstrack_error = a_air % ab;
    float sine_nu1 = constrain_float(crosstrack_error / MAX(deepstall.L1_period, 0.1f), -0.7071f, 0.7107f);
    float nu1 = asinf(sine_nu1);

    if (deepstall.L1_i > 0) {
        deepstall.L1_xtrack_i += nu1 * deepstall.L1_i / dt;
        deepstall.L1_xtrack_i = constrain_float(deepstall.L1_xtrack_i, -0.5f, 0.5f);
        nu1 += deepstall.L1_xtrack_i;
    }

    float desired_change = wrap_PI(radians(deepstall.target_heading_deg) + nu1 - ahrs.yaw);

    float yaw_rate = ahrs.get_gyro().z;
    float yaw_rate_limit = radians(deepstall.yaw_rate_limit);
    float error = wrap_PI(constrain_float(desired_change / deepstall.time_constant,
                                          -yaw_rate_limit, yaw_rate_limit) - yaw_rate);

#ifdef DEBUG_PRINTS
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "x: %f e: %f r: %f d: %f",
                                    (double)crosstrack_error,
                                    (double)error,
                                    (double)degrees(yaw_rate),
                                    (double)location_diff(current_loc, deepstall.landing_point).length());
#endif // DEBUG_PRINTS

    return deepstall.ds_PID.get_pid(error);
}
