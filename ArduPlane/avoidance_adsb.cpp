
#include <stdio.h>
#include "Plane.h"

void Plane::avoidance_adsb_update(void)
{
    adsb.update();
    avoidance_adsb.update();
}


MAV_COLLISION_ACTION AP_Avoidance_Plane::handle_avoidance(const AP_Avoidance::Obstacle *obstacle, MAV_COLLISION_ACTION requested_action)
{
    MAV_COLLISION_ACTION actual_action = requested_action;
    bool failsafe_state_change = false;

    // check for changes in failsafe
    if (!plane.failsafe.adsb) {
        plane.failsafe.adsb = true;
        failsafe_state_change = true;
        // record flight mode in case it's required for the recovery
        prev_control_mode = plane.control_mode;
    }

    // take no action in some flight modes
    if (plane.control_mode == MANUAL ||
        (plane.control_mode == AUTO && !plane.auto_state.takeoff_complete) ||
        (plane.flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND) || // TODO: consider allowing action during approach
        plane.control_mode == AUTOTUNE ||
        plane.control_mode == QLAND) {
        actual_action = MAV_COLLISION_ACTION_NONE;
    }

    // take action based on requested action
    switch (actual_action) {

        case MAV_COLLISION_ACTION_RTL:
            if (failsafe_state_change) {
                plane.set_mode(RTL, MODE_REASON_AVOIDANCE);
            }
            break;

        case MAV_COLLISION_ACTION_HOVER:
            if (failsafe_state_change) {
                if (plane.quadplane.is_flying()) {
                    plane.set_mode(QLOITER, MODE_REASON_AVOIDANCE);
                } else {
                    plane.set_mode(LOITER, MODE_REASON_AVOIDANCE);
                }
            }
            break;

        case MAV_COLLISION_ACTION_ASCEND_OR_DESCEND:
            // climb or descend to avoid obstacle
            if (handle_avoidance_vertical(obstacle, failsafe_state_change)) {
                plane.set_guided_WP();
            } else {
                actual_action = MAV_COLLISION_ACTION_NONE;
            }
            break;

        case MAV_COLLISION_ACTION_MOVE_HORIZONTALLY:
            // move horizontally to avoid obstacle
            if (handle_avoidance_horizontal(obstacle, failsafe_state_change)) {
                plane.set_guided_WP();
            } else {
                actual_action = MAV_COLLISION_ACTION_NONE;
            }
            break;

        case MAV_COLLISION_ACTION_MOVE_PERPENDICULAR:
        {
            // move horizontally and vertically to avoid obstacle
            const bool success_vert = handle_avoidance_vertical(obstacle, failsafe_state_change);
            const bool success_hor = handle_avoidance_horizontal(obstacle, failsafe_state_change);
            if (success_vert || success_hor) {
                plane.set_guided_WP();
            } else {
                actual_action = MAV_COLLISION_ACTION_NONE;
            }
        }
            break;

        // unsupported actions and those that require no response
        case MAV_COLLISION_ACTION_NONE:
            return actual_action;
        case MAV_COLLISION_ACTION_REPORT:
        default:
            break;
    }

    if (failsafe_state_change) {
        gcs().send_text(MAV_SEVERITY_ALERT, "Avoid: Performing action: %d", actual_action);
    }

    // return with action taken
    return actual_action;
}

void AP_Avoidance_Plane::handle_recovery(uint8_t recovery_action)
{
    // check we are coming out of failsafe
    if (plane.failsafe.adsb) {
        plane.failsafe.adsb = false;
        gcs().send_text(MAV_SEVERITY_INFO, "Avoid: Resuming with action: %d", recovery_action);

        // restore flight mode if requested and user has not changed mode since
        if (plane.control_mode_reason == MODE_REASON_AVOIDANCE) {
            switch (recovery_action) {

            case AP_AVOIDANCE_RECOVERY_REMAIN_IN_AVOID_ADSB:
                // do nothing, we'll stay in the AVOID_ADSB mode which is guided which will loiter
                break;

            case AP_AVOIDANCE_RECOVERY_RESUME_PREVIOUS_FLIGHTMODE:
                plane.set_mode(prev_control_mode, MODE_REASON_AVOIDANCE_RECOVERY);
                break;

            case AP_AVOIDANCE_RECOVERY_RTL:
                plane.set_mode(RTL, MODE_REASON_AVOIDANCE_RECOVERY);
                break;

            case AP_AVOIDANCE_RECOVERY_RESUME_IF_AUTO_ELSE_LOITER:
                if (prev_control_mode == AUTO) {
                    plane.set_mode(AUTO, MODE_REASON_AVOIDANCE_RECOVERY);
                }
                // else do nothing, same as AP_AVOIDANCE_RECOVERY_LOITER
                break;

            default:
                break;
            } // switch
        }
    }
}

// check flight mode is avoid_adsb
bool AP_Avoidance_Plane::check_flightmode(bool allow_mode_change)
{
    // ensure plane is in avoid_adsb mode
    if (allow_mode_change && plane.control_mode != AVOID_ADSB) {
        plane.set_mode(AVOID_ADSB, MODE_REASON_AVOIDANCE);
    }

    // check flight mode
    return (plane.control_mode == AVOID_ADSB);
}

bool AP_Avoidance_Plane::handle_avoidance_vertical(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change)
{
    // ensure copter is in avoid_adsb mode
     if (!check_flightmode(allow_mode_change)) {
         return false;
     }

     // get best vector away from obstacle
     if (plane.current_loc.alt > obstacle->_location.alt) {
         // should climb
         plane.guided_WP_loc.alt = plane.current_loc.alt + 1000; // set alt demand to be 10m above us, climb rate will be TECS_CLMB_MAX
         return true;

     } else if (plane.current_loc.alt > plane.g.RTL_altitude_cm) {
         // should descend while above RTL alt
         // TODO: consider using a lower altitude than RTL_altitude_cm since it's default (100m) is quite high
         plane.guided_WP_loc.alt = plane.current_loc.alt - 1000; // set alt demand to be 10m below us, sink rate will be TECS_SINK_MAX
         return true;
     }

     return false;
}

bool AP_Avoidance_Plane::handle_avoidance_horizontal(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change)
{
    // ensure plane is in avoid_adsb mode
    if (!check_flightmode(allow_mode_change)) {
        return false;
    }

    // get best vector away from obstacle
    Vector3f velocity_neu;
    if (get_vector_perpendicular(obstacle, velocity_neu)) {
        // remove vertical component
        velocity_neu.z = 0.0f;

        // check for divide by zero
        if (is_zero(velocity_neu.x) && is_zero(velocity_neu.y)) {
            return false;
        }

        // re-normalize
        velocity_neu.normalize();

        // push vector further away.
        velocity_neu *= 10000;

        // set target
        location_offset(plane.guided_WP_loc, velocity_neu.x, velocity_neu.y);
        return true;
    }

    // if we got this far we failed to set the new target
    return false;
}

/*
  see if a test position is outside the radius of a set of obstacles
 */
bool AP_Avoidance_Plane::mission_avoid_loc_ok(const Vector2f &loc, const Vector2f *predicted_loc, uint8_t count)
{
    const float min_dist = _warn_distance_xy;
    const float dist_sq = sq(min_dist);
    for (uint8_t i=0; i<count; i++) {
        if (predicted_loc[i].is_zero()) {
            // timed out entry
            continue;
        }
        if ((loc - predicted_loc[i]).length_squared() <= dist_sq) {
            return false;
        }
    }
    return true;
}

/*
  see if we can follow a direct path to the target from the given point
 */
bool AP_Avoidance_Plane::mission_direct_path_ok(const Vector2f &loc, const Vector2f &target,
                                                const Vector2f *predicted_loc, uint8_t count,
                                                const Vector2f &tgt,
                                                float avoid_sec, float groundspeed)
{
    const float min_dist = _warn_distance_xy;
    const float dist_sq = sq(min_dist);
    Vector2f loc2 = loc + (tgt - loc).normalized() * groundspeed * avoid_sec;
    for (uint8_t i=0; i<count; i++) {
        if (predicted_loc[i].is_zero()) {
            // timed out entry
            continue;
        }
        Vector2f pred2 = predicted_loc[i] + Vector2f(_obstacles[i]._velocity.x, _obstacles[i]._velocity.y) * avoid_sec;
        if ((loc2 - pred2).length_squared() <= dist_sq) {
            return false;
        }
    }
    return true;
}

/*
  update waypoint to avoid dynamic obstacles
 */
void AP_Avoidance_Plane::mission_avoidance(const Location &current_loc, Location &target_loc, float groundspeed)
{
    if (!_enabled || _warn_action != 2) {
        return;
    }
    const float avoid_ratio = 4;
    const float avoid_sec = (_warn_distance_xy / groundspeed) / avoid_ratio;
    const int32_t bearing_inc_cd = 500;
    const float full_distance = get_distance(current_loc, target_loc);
    const float distance = groundspeed * avoid_sec;
    const int32_t bearing_cd = get_bearing_cd(current_loc, target_loc);
    const uint32_t timeout_ms = 5000;

    // get predicted locations of all obstacles
    const uint32_t now = AP_HAL::millis();
    Vector2f predicted_loc[_obstacle_count] {};
    
    for (uint8_t i=0; i<_obstacle_count; i++) {
        if (now - _obstacles[i].timestamp_ms > timeout_ms) {
            continue;
        }
        Vector2f vel(_obstacles[i]._velocity.x, _obstacles[i]._velocity.y);
        predicted_loc[i] = location_diff(current_loc, _obstacles[i]._location);
        predicted_loc[i] += vel * avoid_sec;
    }

    const Vector2f target = location_diff(current_loc, target_loc);

    for (uint8_t i=0; i<72; i++) {
        int32_t bearing_delta_cd = i*bearing_inc_cd/2;
        if (i & 1) {
            bearing_delta_cd = -bearing_delta_cd;
        }
        const float bearing_test = (bearing_cd + bearing_delta_cd)*0.01;
        Vector2f loc_test = Vector2f(cosf(radians(bearing_test)), sinf(radians(bearing_test))) * distance;
        if (mission_avoid_loc_ok(loc_test, predicted_loc, _obstacle_count)) {
            // found a good bearing, double check it will allow for a direct path after full radius
            Vector2f loc_test2 = Vector2f(cosf(radians(bearing_test)), sinf(radians(bearing_test))) * distance * avoid_ratio;
            if (mission_direct_path_ok(loc_test2, target, predicted_loc, _obstacle_count, target, avoid_sec / avoid_ratio, groundspeed)) {
                target_loc = current_loc;
                location_update(target_loc, bearing_test, full_distance);
                if (i != 0) {
                    ::printf("avoid bearing %d\n", bearing_delta_cd);
                }
                return;
            }
        }
    }
}
