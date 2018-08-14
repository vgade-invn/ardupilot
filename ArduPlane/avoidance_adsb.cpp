
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
    for (uint8_t i=0; i<count; i++) {
        if (predicted_loc[i].is_zero()) {
            // timed out entry
            continue;
        }
        if (!within_avoidance_height(_obstacles[i])) {
            continue;
        }
        const Vector2f &ploc = predicted_loc[i];

        // lookup how far to keep from the object
        const float radius = get_avoidance_radius(_obstacles[i]) + exclusion_margin;

        // optimisation to short-circuit most checks
        unsigned short_circuit = 0;
        if (loc.x > 0 && ploc.x > 0 && ploc.x - loc.x > radius) {
            short_circuit = __LINE__;
        }
        if (loc.x < 0 && ploc.x < 0 && loc.x - ploc.x > radius) {
            short_circuit = __LINE__;
        }
        if (loc.y > 0 && ploc.y > 0 && ploc.y - loc.y > radius) {
            short_circuit = __LINE__;
        }
        if (loc.y < 0 && ploc.y < 0 && loc.y - ploc.y > radius) {
            short_circuit = __LINE__;
        }
        Vector2f origin {};
        Vector2f tmp;
        if (Vector2f::circle_segment_intersection(origin, loc, predicted_loc[i], radius, tmp)) {
            if (short_circuit) {
                AP_HAL::panic("short circuit error");
            }
            return false;
        }
    }
    return true;
}

/*
  update waypoint to avoid dynamic obstacles. Return true if doing avoidance
 */
bool AP_Avoidance_Plane::mission_avoidance(const Location &current_loc, Location &target_loc, float groundspeed)
{
    if (!_enabled || _warn_action != 2) {
        return false;
    }
    const float avoid_ratio = 4;
    const float full_distance = get_distance(current_loc, target_loc);
    const float avoid_max = MIN(_warn_distance_xy, full_distance);
    const float avoid_sec = (avoid_max / groundspeed) / avoid_ratio;
    const int32_t bearing_inc_cd = 500;
    const float distance = groundspeed * avoid_sec;
    const int32_t bearing_cd = get_bearing_cd(current_loc, target_loc);
    const uint32_t timeout_ms = 5000;

    if (full_distance < 50) {
        // if we are within 50m, go for it
        return false;
    }

    // load exclusion zones from the mission, if any
    load_exclusion_zones();

    // load inner fence, if any
    load_fence_boundary();
    
    // get predicted locations of all obstacles
    const uint32_t now = AP_HAL::millis();
    Vector2f predicted_loc[_obstacle_count] {};

    // predict forward the positions of the moving obstacles
    for (uint8_t i=0; i<_obstacle_count; i++) {
        if (now - _obstacles[i].timestamp_ms > timeout_ms) {
            continue;
        }
        Vector2f vel(_obstacles[i]._velocity.x, _obstacles[i]._velocity.y);
        predicted_loc[i] = location_diff(current_loc, _obstacles[i]._location);
        predicted_loc[i] += vel * avoid_sec;
    }

    // try all 5 degree increments around a circle, alternating left
    // and right. For each one check that if we flew in that direction
    // that we would avoid all obstacles
    float best_bearing = bearing_cd * 0.01;
    bool have_best_bearing = false;
    
    for (uint8_t i=0; i<360 / (bearing_inc_cd/100); i++) {
        int32_t bearing_delta_cd = i*bearing_inc_cd/2;
        if (i & 1) {
            bearing_delta_cd = -bearing_delta_cd;
        }
        // bearing that we are probing
        const float bearing_test = (bearing_cd + bearing_delta_cd)*0.01;

        // position we will get to
        Location loc_test = current_loc;
        location_update(loc_test, bearing_test, distance);

        // difference from current_loc as floats
        Vector2f loc_diff = location_diff(current_loc, loc_test);
        
        if (mission_avoid_loc_ok(loc_diff, predicted_loc, _obstacle_count) &&
            mission_avoid_exclusions(current_loc, loc_test) &&
            mission_avoid_fence(loc_test)) {
            // This bearing will avoid all dynamic and static
            // obstacles for one step of 'distance'. Now check if it
            // will put us in a position where we have a clear path to
            // our destination for distance*avoid_ratio
            if (!have_best_bearing) {
                best_bearing = bearing_test;
                have_best_bearing = true;
            }
            
            float new_bearing = 0.01 * get_bearing_cd(loc_test, target_loc);
            Location loc_test2 = loc_test;
            location_update(loc_test2, new_bearing, distance * avoid_ratio);
            Vector2f loc_diff2 = location_diff(current_loc, loc_test2);
            if (mission_avoid_loc_ok(loc_diff2, predicted_loc, _obstacle_count) &&
                mission_avoid_exclusions(loc_test, loc_test2) &&
                mission_avoid_fence(loc_test2)) {
                // all good, now project in the chosen direction by the full distance
                target_loc = current_loc;
                location_update(target_loc, bearing_test, full_distance);
                return i != 0;
            }
        }
    }

    // none of the possible paths were OK for our tests, choose the
    // one that did best on the first step
    target_loc = current_loc;
    location_update(target_loc, best_bearing, full_distance);
    
    return true;
}

/*
  load exclusion zones from mission items
 */
void AP_Avoidance_Plane::load_exclusion_zones(void)
{
    if (mission_change_ms == plane.mission.last_change_time_ms()) {
        // no change
        return;
    }

    // unload previous zones
    unload_exclusion_zones();
    
    num_exclusion_zones = plane.mission.get_fence_exclusion_count();
    if (num_exclusion_zones == 0) {
        return;
    }
    exclusion_zones = new struct exclusion_zone [num_exclusion_zones];
    if (!exclusion_zones) {
        goto failed;
    }
    for (uint8_t zone=0; zone<num_exclusion_zones; zone++) {
        struct exclusion_zone &ezone = exclusion_zones[zone];
        uint16_t count;
        uint16_t start = plane.mission.get_fence_exclusion_start(zone, count);
        if (start == 0) {
            goto failed;
        }
        ezone.num_points = count+1;
        ezone.points = new Vector2f[ezone.num_points];
        if (!ezone.points) {
            goto failed;
        }
        for (uint8_t i=0; i<count; i++) {
            AP_Mission::Mission_Command cmd;
            if (!plane.mission.read_cmd_from_storage(start+i, cmd)) {
                goto failed;
            }
            ezone.points[i].x = cmd.content.fence_vertex.lat*1e-7;
            ezone.points[i].y = cmd.content.fence_vertex.lng*1e-7;
        }
        // complete the polygon, so we can use the polygon library
        // functions
        ezone.points[count] = ezone.points[0];

        // expand it by the margin
        grow_polygonf(ezone.points, ezone.num_points, exclusion_margin);
    }
    mission_change_ms = plane.mission.last_change_time_ms();
    return;
    
failed:
    gcs().send_text(MAV_SEVERITY_CRITICAL, "exclusion zone failed");
    
    // set the timestamp so we don't continuously reload
    mission_change_ms = plane.mission.last_change_time_ms();
}

/*
  unload the exclusion zones
 */
void AP_Avoidance_Plane::unload_exclusion_zones(void)
{
    if (!exclusion_zones) {
        num_exclusion_zones = 0;
        return;
    }
    for (uint8_t zone=0; zone<num_exclusion_zones; zone++) {
        delete [] exclusion_zones[zone].points;
    }
    delete exclusion_zones;
    exclusion_zones = nullptr;
}

/*
  return true if the path from current_loc to loc_test avoids all exclusion zones
  NOTE: should also avoid the geofence
 */
bool AP_Avoidance_Plane::mission_avoid_exclusions(const Location &current_loc, const Location &loc_test)
{
    for (uint8_t zone=0; zone<num_exclusion_zones; zone++) {
        const struct exclusion_zone &ezone = exclusion_zones[zone];
        Vector2f p1(current_loc.lat*1e-7, current_loc.lng*1e-7);
        Vector2f p2(loc_test.lat*1e-7, loc_test.lng*1e-7);
        if (Polygon_intersects(ezone.points, ezone.num_points, p1, p2)) {
            return false;
        }
    }
    return true;
}

/*
  grow a Vector2f polygon by a given amount. Assumes polygon is in decimal degrees
 */
void AP_Avoidance_Plane::grow_polygonf(Vector2f *points, uint8_t num_points, float change_m)
{
    // find center
    Vector2f center;
    for (uint8_t i=0; i<num_points-1; i++) {
        center += points[i];
    }
    center /= num_points - 1;
    Location center_loc;
    center_loc.lat = center.x * 1e7;
    center_loc.lng = center.y * 1e7;

    // move each point away from the center by the margin
    for (uint8_t i=0; i<num_points-1; i++) {
        Vector2f &pt = points[i];
        Location loc;
        loc.lat = pt.x * 1e7;
        loc.lng = pt.y * 1e7;
        
        float distance = get_distance(center_loc, loc);
        float bearing = get_bearing_deg(center_loc, loc);
        loc = center_loc;
        location_update(loc, bearing, MAX(distance+change_m, 1));
        pt.x = loc.lat * 1e-7;
        pt.y = loc.lng * 1e-7;
    }

    // close the loop again
    points[num_points-1] = points[0];
}

/*
  grow an exclusion zone by a given amount, Vector2l version. Assumes
  polygon is in degrees * 1e7
 */
void AP_Avoidance_Plane::grow_polygonl(Vector2l *points, uint8_t num_points, float change_m)
{
    // find center
    Vector2f center;
    for (uint8_t i=0; i<num_points-1; i++) {
        center.x += points[i].x;
        center.y += points[i].y;
    }
    center /= num_points - 1;
    Location center_loc;
    center_loc.lat = center.x;
    center_loc.lng = center.y;

    // move each point away from the center by the margin
    for (uint8_t i=0; i<num_points-1; i++) {
        Vector2l &pt = points[i];
        Location loc;
        loc.lat = pt.x;
        loc.lng = pt.y;
        
        float distance = get_distance(center_loc, loc);
        float bearing = get_bearing_deg(center_loc, loc);
        loc = center_loc;
        location_update(loc, bearing, MAX(distance+change_m, 1));
        pt.x = loc.lat;
        pt.y = loc.lng;
    }

    // close the loop again
    points[num_points-1] = points[0];
}

/*
  get the avoidance radius in meters of a given obstacle type
 */
float AP_Avoidance_Plane::get_avoidance_radius(const class Obstacle &obstacle) const
{
    if (obstacle.src_id < 20000) {
        // fixed wing, 300m radius
        return 300;
    }
    if (obstacle.src_id < 30000) {
        // weather, radius 150 at ground, 300m at 3000m, 173m at 1500ft
        return 173;
    }
    if (obstacle.src_id < 40000) {
        // migratory bird, 100m
        return 100;
    }
    if (obstacle.src_id < 50000) {
        // bird of prey, 200m
        return 200;
    }
    // default to 300, which is worst case
    return 300;
}

/*
  check if we are within the height range to need to avoid an obstacle
 */
bool AP_Avoidance_Plane::within_avoidance_height(const class Obstacle &obstacle) const
{
    if (obstacle.src_id >= 20000 && obstacle.src_id < 30000) {
        // weather, always avoid
        return true;
    }
    int32_t alt_cm;
    const Location_Class &loc = obstacle._location;
    if (!loc.get_alt_cm(Location_Class::ALT_FRAME_ABSOLUTE, alt_cm)) {
        return true;
    }
    float obstacle_alt = alt_cm * 0.01;
    float alt_min, alt_max;
    const float margin = 20;
    
    if (obstacle.src_id < 20000 || (obstacle.src_id >= 30000 && obstacle.src_id < 40000)) {
        // fixed wing or migrating bird, height range 150m, +10 seconds of height change
        alt_min = obstacle_alt - (75+margin);
        alt_max = obstacle_alt + (75+margin);
    } else {
        // bird of prey, from location to ground
        alt_max = obstacle_alt + margin;
        alt_min = 0;
    }
    if (obstacle._velocity.z < 0) {
        alt_max -= 10 * obstacle._velocity.z;
    } else {
        alt_min -= 10 * obstacle._velocity.z;
    }
    const Location_Class &myloc = plane.current_loc;
    if (!myloc.get_alt_cm(Location_Class::ALT_FRAME_ABSOLUTE, alt_cm)) {
        return true;
    }
    float alt = alt_cm * 0.01;
    // are we in the range of avoidance heights?
    return alt > alt_min && alt < alt_max;
}

/*
  load fence boundary with an avoidance margin
 */
void AP_Avoidance_Plane::load_fence_boundary(void)
{
    if (plane.geofence_last_change_ms() == last_fence_change_ms) {
        return;
    }
    last_fence_change_ms = plane.geofence_last_change_ms();

    // clear old fence
    if (fence_points) {
        delete [] fence_points;
        fence_points = nullptr;
        num_fence_points = 0;
    }

    Vector2l *points;
    uint8_t num_points;
    if (!plane.geofence_get_polygon(points, num_points) || num_points < 4) {
        // no fence to load
        return;
    }

    // copy the fence, so we can expand it
    fence_points = new Vector2l[num_points];
    if (fence_points == nullptr) {
        return;
    }
    memcpy(fence_points, points, num_points * sizeof(fence_points[0]));
    num_fence_points = num_points;

    // reduce its size by a margin of 100m
    grow_polygonl(fence_points, num_points, -fence_margin);
}

/*
  does the proposed flight path avoid the fence?
 */
bool AP_Avoidance_Plane::mission_avoid_fence(const Location &loc_test)
{
    if (num_fence_points < 4) {
        return true;
    }
    Vector2l loc;
    loc.x = loc_test.lat;
    loc.y = loc_test.lng;
    return !Polygon_outside(loc, fence_points, num_fence_points);
}
