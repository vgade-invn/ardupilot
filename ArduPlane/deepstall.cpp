#include "deepstall.h"
#include <math.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Navigation/AP_Navigation.h>

#define BOOL_TO_SIGN(bvalue) ((bvalue) ? -1 : 1)
extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo DeepStall::var_info[] = {
    // @Param: VF_A
    // @DisplayName: GPS type
    // @Description: GPS type
    // @Values: 0:None,1:AUTO,2:uBlox,3:MTK,4:MTK19,5:NMEA,6:SiRF,7:HIL,8:SwiftNav,9:PX4-UAVCAN,10:SBF,11:GSOF
    // @RebootRequired: True
    AP_GROUPINFO("VF_A",    0, DeepStall, vf_a, -1.179490546),

    // @Param: VF_B
    // @DisplayName: 2nd GPS type
    // @Description: GPS type of 2nd GPS
    // @Values: 0:None,1:AUTO,2:uBlox,3:MTK,4:MTK19,5:NMEA,6:SiRF,7:HIL,8:SwiftNav,9:PX4-UAVCAN,10:SBF,11:GSOF
    // @RebootRequired: True
    AP_GROUPINFO("VF_B",   1, DeepStall, vf_b, 11.22342218),

    // @Param: VF_B
    // @DisplayName: 2nd GPS type
    // @Description: GPS type of 2nd GPS
    // @Values: 0:None,1:AUTO,2:uBlox,3:MTK,4:MTK19,5:NMEA,6:SiRF,7:HIL,8:SwiftNav,9:PX4-UAVCAN,10:SBF,11:GSOF
    // @RebootRequired: True
    AP_GROUPINFO("TCON",   2, DeepStall, tcon, 2.647),

    // @Param: DS_A
    // @DisplayName: GPS type
    // @Description: GPS type
    // @Values: 0:None,1:AUTO,2:uBlox,3:MTK,4:MTK19,5:NMEA,6:SiRF,7:HIL,8:SwiftNav,9:PX4-UAVCAN,10:SBF,11:GSOF
    // @RebootRequired: True
    AP_GROUPINFO("DS_A",    3, DeepStall, ds_a, -1.486634076),

    // @Param: DS_B
    // @DisplayName: 2nd GPS type
    // @Description: GPS type of 2nd GPS
    // @Values: 0:None,1:AUTO,2:uBlox,3:MTK,4:MTK19,5:NMEA,6:SiRF,7:HIL,8:SwiftNav,9:PX4-UAVCAN,10:SBF,11:GSOF
    // @RebootRequired: True
    AP_GROUPINFO("DS_B",   4, DeepStall, ds_b, 16.28549267),

    AP_GROUPEND
};


DeepStall::DeepStall() {
    YawRateController = new PIDController(0,0,0);
    YawRateController->setIntegralLimit(0);
    rCmd = 0;
    targetHeading = 0;
    _last_t = 0;
    d_predict = 0;
    ds = 100;
    stage = DEEPSTALL_FLY_TO_LOITER;
    ready = false;
    AP_Param::setup_object_defaults(this, var_info);
}

void DeepStall::abort() {
	YawRateController->resetIntegrator();
	stage = DEEPSTALL_FLY_TO_LOITER; // Reset deepstall stage in case of abort
	ready = false;
	_last_t = 0;
        loiter_sum_cd = 0;
}

void DeepStall::setYRCParams(float _Kyr, float _yrLimit, float Kp, float Ki, float Kd, float ilim) {
    YawRateController->setGains(Kp, Ki, Kd);
    YawRateController->setIntegralLimit(ilim);
    yrLimit = _yrLimit;
}

float DeepStall::predictDistanceTraveled(Vector3f wind, float altitude, float vspeed) {
    float forward_distance = vf_a * wind.length() + vf_b;
    float stall_distance = ds_a * wind.length() + ds_b;
    float course = targetHeading * M_PI/180;

    Vector3f aligned_component(sin(course), cos(course), 0.0f);
    float v_e = forward_distance;// * (aligned_component * wind);
    return v_e * altitude / vspeed + stall_distance;
}

void DeepStall::computeApproachPath(Vector3f _wind, float loiterRadius, float d_s, float v_d, float deltah, float vspeed, Location &landing, float heading) {

        memcpy(&landing_point, &landing, sizeof(Location));
        memcpy(&extended_approach, &landing, sizeof(Location));
        memcpy(&loiter_exit, &landing, sizeof(Location));


        // extended approach point is 1km away so that there is a navigational target
        location_update(extended_approach, targetHeading, 1000.0);

	ds = d_s;

	float course = targetHeading*M_PI/180;

	// Generate v_d and wind vectors
	Vector2f Vd(v_d*sin(course), v_d*cos(course));
	Vector2f wind(_wind.y, _wind.x);
	
	// Compute effective groundspeed - can be negative, hence can handle backward tracking
	float v_e = (1/v_d)*(Vd * (Vd + wind)); // should essentially do dot(Vd,Vd+wind)/v_d
	
	// Predict deepstall distance (can handle backward tracking! xD)
	d_predict = v_e*deltah/vspeed;
        d_predict = predictDistanceTraveled(_wind, deltah, v_d);
	
        location_update(loiter_exit, targetHeading + 180.0, d_predict + loiterRadius + d_s);
        memcpy(&loiter, &loiter_exit, sizeof(Location));
        location_update(loiter, targetHeading + 90.0, loiterRadius);

        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
	                                 "Loiter: %3.8f %3.8f\n", loiter.lat / 1e7, loiter.lng / 1e7);
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
	                                 "Loiter exit: %3.8f %3.8f\n", loiter_exit.lat / 1e7, loiter_exit.lng / 1e7);
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
	                                 "Landing: %3.8f %3.8f\n", landing.lat / 1e7, landing.lng / 1e7);
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
	                                 "Extended: %3.8f %3.8f\n", extended_approach.lat / 1e7, extended_approach.lng / 1e7);
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
                                         "Extended by: %f\n", d_predict + loiterRadius + d_s);
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
	                                 "Wind Heading: %3.1f\n\n", targetHeading);
	
}

bool DeepStall::verify_loiter_breakout(Location &current_loc, int32_t heading_cd) {
    // Bearing in degrees
    int32_t bearing_cd = get_bearing_cd(current_loc, extended_approach);

    int32_t heading_err_cd = wrap_180_cd(bearing_cd - heading_cd);

    /*
      Check to see if the the plane is heading toward the land
      waypoint. We use 20 degrees (+/-10 deg) of margin so that
      we can handle 200 degrees/second of yaw. We also require
      the altitude to be within 0.5 meters of desired, and
      enforce a minimum of one turn
    */
    if (loiter_sum_cd > 18000 &&
        labs(heading_err_cd) <= 1000  &&
        labs(loiter.alt - current_loc.alt) < 500) {
        // Want to head in a straight line from _here_ to the next waypoint instead of center of loiter wp
        return true;
    }
    return false;
}

STAGE DeepStall::getApproachWaypoint(Location &target, Location &land_loc, Location &current_loc, Vector3f _wind, float v_d, float deltah, float vspeed, int32_t heading_cd, AP_Navigation *nav_controller, float loiter_radius, float heading) {

    // fly to the loiter point if we are to far away
    if (stage == DEEPSTALL_FLY_TO_LOITER && get_distance(current_loc, land_loc) > 500) {
        memcpy(&target, &loiter, sizeof(Location));
    } else {

        float course = targetHeading*M_PI/180;

        // Generate v_d and wind vectors
        Vector2f Vd(v_d*sin(course), v_d*cos(course));
        Vector2f wind(_wind.y, _wind.x);

        // Compute effective groundspeed - can be negative, hence can handle backward tracking
        float v_e = (1/v_d)*(Vd * (Vd + wind)); // should essentially do dot(Vd,Vd+wind)/v_d

        // Predict deepstall distance (can handle backward tracking! xD)
        d_predict = v_e*deltah/vspeed;
        d_predict = predictDistanceTraveled(_wind, deltah, vspeed);
	
        switch (stage) {
            case DEEPSTALL_FLY_TO_LOITER:
                if (get_distance(current_loc, loiter) > 2 * loiter_radius) {
                    memcpy(&target, &loiter, sizeof(Location));
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
                                                     "Fly to loiter: d: %f\n", get_distance(current_loc, loiter));
                    break;
                } else {
                    // when within twice the loiter radius, fall through to loiter
                    stage = DEEPSTALL_LOITER;
                }
            case DEEPSTALL_LOITER:
                // fly at the point until it's been reached
                if (!nav_controller->reached_loiter_target()) {
                    memcpy(&target, &loiter, sizeof(Location));
                    old_target_bearing_cd = nav_controller->target_bearing_cd();
                    loiter_sum_cd = 0;
                    break;
                } else {
                    // update the loiter progress
                    loiter_sum_cd += wrap_180_cd(nav_controller->target_bearing_cd() - old_target_bearing_cd);
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
                                                     "Loiter: cd: %d\n", loiter_sum_cd);
                    old_target_bearing_cd = nav_controller->target_bearing_cd();
                    if (verify_loiter_breakout(current_loc, heading_cd)) {
                        // breakout of the loiter
                        stage = DEEPSTALL_APPROACH;
                        if (heading == 0.0) {
                            setTargetHeading(atan2(-_wind.y, -_wind.x) * 180 / M_PI, true);
                        }
                        computeApproachPath(_wind, loiter_radius, 100.0f, v_d, deltah, vspeed, land_loc, heading);
                    } else {
                        memcpy(&target, &loiter, sizeof(Location));
                        break;
                    }
                }
            case DEEPSTALL_APPROACH:
                // always fly at the extended approach point
                memcpy(&target, &extended_approach, sizeof(Location));
                // check if we should enter the stall
                Location entry_loc;
                memcpy(&entry_loc, &landing_point, sizeof(Location));
                location_update(entry_loc, targetHeading + 180.0f, d_predict);
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Approach: l: %f p: %f d: %f\n",
                                                 get_distance(current_loc, land_loc),
                                                 d_predict,
                                                 get_distance(current_loc, entry_loc));
                if (location_passed_point(current_loc, loiter_exit, entry_loc)) {
                    stage = DEEPSTALL_LAND;
                } else {
                    if (location_passed_point(current_loc, loiter, extended_approach)){
                        stage = DEEPSTALL_FLY_TO_LOITER;
                        loiter_sum_cd = 0;
                    }
                    break;
                }
            case DEEPSTALL_LAND:
                memcpy(&target, &extended_approach, sizeof(Location));
                break;
        }
    }
    return stage;
}

void DeepStall::land(float track, float yawrate, Location current_loc, float deepstall_l1, float yawlimit) {

	uint32_t tnow = AP_HAL::millis();
	uint32_t dt = tnow - _last_t;
	if (_last_t == 0 || dt > 1000) {
		dt = 10; // Default to 100 Hz
	}
	_last_t = tnow;

	// Target position controller
	// Generate equation of the tracking line parameters
	float course = targetHeading*M_PI/180;
	
        Vector2f ab = location_diff(loiter_exit, extended_approach);
        ab.normalize();
        Vector2f a_air = location_diff(loiter_exit, current_loc);

	float _crosstrack_error = a_air % ab;
        float sine_nu1 = _crosstrack_error / MAX(deepstall_l1, 0.1f);
        sine_nu1 = constrain_float(sine_nu1, -0.7071f, 0.7107f);
        float nu1 = asinf(sine_nu1);

	targetTrack = course + nu1;
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
                   "%f %f %f %f %f\n",
                    _crosstrack_error,
                    CLAMP((targetTrack - track) *0.05/0.3f, -yawlimit, yawlimit), 
                    180 / M_PI * nu1,
                    yawrate * 180 / M_PI,
                    location_diff(current_loc, landing_point).length());
		
	rCmd = PIDController::saturate(
                 YawRateController->run(((float) dt)/1000.0,
                                        PIDController::saturate(
                                          PIDController::wrap(CLAMP((targetTrack - track) / tcon, -yawlimit, yawlimit) - yawrate,  -M_PI, M_PI), -yrLimit, yrLimit)),
                 -1, 1);
}

float DeepStall::getRudderNorm() {
	return rCmd;
}

void DeepStall::setTargetHeading(float hdg, bool constrain) {
        if (constrain) {
            float delta = hdg - targetHeading;
            delta += (delta > 180.0f) ? -360.0f : (delta < -180.0f) ? 360.0f : 0.0f;
            targetHeading = PIDController::wrap(CLAMP(delta, -15.0f, 15.0f) + targetHeading, -180.0f, 180.0f);
        } else {
            targetHeading = hdg;
        }
}
