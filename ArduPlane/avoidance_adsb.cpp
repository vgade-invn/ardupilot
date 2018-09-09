#include <stdio.h>
#include "Plane.h"

#define debug(level, fmt, args ...) do { if (_debug.get()>=(level)) { gcs().send_text(MAV_SEVERITY_INFO, "AVD: " fmt, ## args); } } while (0)

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
  given our location, velocity and a avoidance time, find the closest
  we will come to any of the obstacles avoidance radiuses
  A negative result means we will come within the avoidance radius of at least one object
 */
float AP_Avoidance_Plane::mission_avoidance_margin(const Location &our_loc, const Vector2f &our_velocity, float avoid_sec)
{
    const uint32_t timeout_ms = 5000;
    float margin = MAX(_margin_wide, _warn_distance_xy);
    uint8_t num_outside_height_range = 0;
    uint8_t num_timed_out = 0;
    float closest_dist = 0;
    uint8_t closest_id = 0;

    gcs_threat.closest_approach_z = 1000;

    for (uint8_t i=0; i<_obstacle_count; i++) {
        // obstacles can update via MAVLink while we and calculating
        // avoidance margins
        Obstacle obstacle;
        {
            WITH_SEMAPHORE(_rsem);
            obstacle = _obstacles[i];
        }
        uint32_t now = AP_HAL::millis();
        if (now - obstacle.timestamp_ms > timeout_ms) {
            num_timed_out++;
            continue;
        }
        if (!within_avoidance_height(obstacle, _margin_height, avoid_sec)) {
            num_outside_height_range++;
            continue;
        }

        // use our starting point as origin
        Vector2f obstacle_position = location_diff(our_loc, obstacle._location);

        // update obstacle position by delta time since we logged its position
        Vector2f obstacle_velocity(obstacle._velocity.x,obstacle._velocity.y);
        float dt = (now - obstacle.timestamp_ms) * 0.001;
        obstacle_position += obstacle_velocity * dt;
        
        // get our velocity relative to obstacle
        const Vector2f relative_velocity = our_velocity - obstacle_velocity;
        const Vector2f final_pos = relative_velocity * avoid_sec;

        // lookup the min distance to keep from the object
        const float radius = get_avoidance_radius(obstacle);

        // assume that messages about aircraft position could be up to 2s old when we get them
        const float position_lag = 2.0;
        float position_error = position_lag * obstacle_velocity.length();

        float dist = Vector2f::closest_distance_between_radial_and_point(final_pos, obstacle_position) - (radius + position_error);
        dist -= _margin_dynamic;
        if (dist < margin) {
            margin = dist;
            closest_dist = dist;
            closest_id = i;
        }
    }

    if (closest_dist > 0) {
        // update threat report for GCS
        Obstacle obstacle;
        {
            WITH_SEMAPHORE(_rsem);
            obstacle = _obstacles[closest_id];
        }
        gcs_threat.src_id = obstacle.src_id;
        gcs_threat.threat_level = MAV_COLLISION_THREAT_LEVEL_LOW;
        gcs_threat.time_to_closest_approach = 0;
        gcs_threat.closest_approach_xy = closest_dist + _margin_dynamic;
        gcs_threat.closest_approach_z = obstacle_height_difference(obstacle);
    }

    
    debug(3, "margin %.1f to:%u oh:%u t:%u\n", margin, num_timed_out, num_outside_height_range, _obstacle_count);
    return margin;
}

/*
  return the minimum distance we would come to an exclusion zone when
  flying from current_loc to loc_test
  A negative result means we would come within exclusion_margin_static
  meters of a exclusion zone
 */
float AP_Avoidance_Plane::mission_exclusion_margin(const Location &current_loc, const Location &loc_test)
{
    float margin = MAX(_margin_wide, _warn_distance_xy);
    for (uint8_t zone=0; zone<num_exclusion_zones; zone++) {
        const struct exclusion_zone &ezone = exclusion_zones[zone];
        const Vector2f p1 = location_diff(ezone.first_loc, current_loc);
        const Vector2f p2 = location_diff(ezone.first_loc, loc_test);
        float dist = Polygon_closest_distance_line(ezone.points, ezone.num_points, p1, p2) - _margin_exclusion;
        if (dist < margin) {
            margin = dist;
        }
    }
    return margin;
}

/*
  see if we have clear airspace for a given distance and time
 */
bool AP_Avoidance_Plane::mission_clear(const Location &current_loc, float xy_clearance, float z_clearance, float time_s)
{
    if (!_enabled || _warn_action != 2) {
        return true;
    }

    const uint32_t timeout_ms = 5000;

    WITH_SEMAPHORE(_rsem);

    uint32_t now = AP_HAL::millis();

    // assume we are not moving
    Vector3f my_vel;

    for (uint8_t i=0; i<_obstacle_count; i++) {
        const Obstacle &obstacle = _obstacles[i];
        if (now - obstacle.timestamp_ms > timeout_ms) {
            continue;
        }
        if (!within_avoidance_height(obstacle, z_clearance, time_s)) {
            continue;
        }

        // get updated obstacle position
        Location obstacle_loc = obstacle._location;
        Vector2f obstacle_velocity(obstacle._velocity.x,obstacle._velocity.y);
        float dt = (now - obstacle.timestamp_ms) * 0.001;
        location_offset(obstacle_loc, obstacle_velocity.x * dt, obstacle_velocity.y * dt);

        float closest_xy = closest_approach_xy(current_loc, my_vel, obstacle_loc, obstacle._velocity, time_s);

        const float radius = get_avoidance_radius(obstacle);

        if (closest_xy < xy_clearance + radius) {
            // it could come within the radius in the given time
            gcs_threat.src_id = obstacle.src_id;
            gcs_threat.threat_level = MAV_COLLISION_THREAT_LEVEL_HIGH;
            gcs_threat.time_to_closest_approach = 0;
            gcs_threat.closest_approach_xy = closest_xy - radius;
            gcs_threat.closest_approach_z = obstacle_height_difference(obstacle);
            gcs_action = (MAV_COLLISION_ACTION)1;
            return false;
        }
    }

    // all clear
    return true;
}


/*
  check if we have already collided with a dynamic obstacle
 */
bool AP_Avoidance_Plane::have_collided(const Location &current_loc)
{
    const uint32_t timeout_ms = 5000;
    bool ret = false;

    WITH_SEMAPHORE(_rsem);
    
    uint32_t now = AP_HAL::millis();

    for (uint8_t i=0; i<_obstacle_count; i++) {
        const Obstacle &obstacle = _obstacles[i];
        if (now - obstacle.timestamp_ms > timeout_ms) {
            continue;
        }
        if (!within_avoidance_height(obstacle, 0, 0)) {
            continue;
        }

        // get updated obstacle position
        Location obstacle_loc = obstacle._location;
        Vector2f obstacle_velocity(obstacle._velocity.x,obstacle._velocity.y);
        float dt = (now - obstacle.timestamp_ms) * 0.001;
        location_offset(obstacle_loc, obstacle_velocity.x * dt, obstacle_velocity.y * dt);
        
        const float radius = get_avoidance_radius(obstacle);
        float distance = get_distance(current_loc, obstacle_loc);

        if (distance < radius) {
            debug(1, "Collided with %u %.0fm\n", obstacle.src_id, distance);
            ret = true;
        }
    }

    for (uint8_t zone=0; zone<num_exclusion_zones; zone++) {
        const struct exclusion_zone &ezone = exclusion_zones[zone];
        const Vector2f p1 = location_diff(ezone.first_loc, current_loc);
        if (!Polygon_outside(p1, ezone.points, ezone.num_points)) {
            debug(1, "Within exclusion wp:%u\n", ezone.first_wp);
            ret = true;
        }
    }

    collision_detected = ret;
    
    return ret;
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

    WITH_SEMAPHORE(plane.mission.get_semaphore());

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
            Location loc;
            loc.lat = cmd.content.fence_vertex.lat;
            loc.lng = cmd.content.fence_vertex.lng;
            if (i == 0) {
                ezone.first_loc = loc;
                ezone.first_wp = start;
            }
            ezone.points[i] = location_diff(ezone.first_loc, loc);
        }
        // complete the polygon, so we can use the polygon library
        // functions
        ezone.points[count] = ezone.points[0];
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
bool AP_Avoidance_Plane::within_avoidance_height(const class Obstacle &obstacle, const float margin, float deltat) const
{
    if (_options.get() & OPTION_IGNORE_HEIGHT) {
        return true;
    }
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

    if (obstacle.src_id < 20000 || (obstacle.src_id >= 30000 && obstacle.src_id < 40000)) {
        // fixed wing or migrating bird, height range 150m, deltat seconds of height change
        alt_min = obstacle_alt - (75+margin);
        alt_max = obstacle_alt + (75+margin);
    } else {
        // bird of prey, from location to ground
        alt_max = obstacle_alt + margin;
        alt_min = -10000;
    }
    // note that velocity is NED
    if (obstacle._velocity.z < 0) {
        alt_max -= deltat * obstacle._velocity.z;
    } else {
        alt_min -= deltat * obstacle._velocity.z;
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
    WITH_SEMAPHORE(plane.get_fence_semaphore());

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

    fence_origin.lat = points[0].x;
    fence_origin.lng = points[0].y;

    // copy the fence as a set of floats
    fence_points = new Vector2f[num_points];
    if (fence_points == nullptr) {
        return;
    }
    for (uint8_t i=0; i<num_points; i++) {
        Location loc2;
        loc2.lat = points[i].x;
        loc2.lng = points[i].y;
        fence_points[i] = location_diff(fence_origin, loc2);
    }
    num_fence_points = num_points;
}

/*
  by how much over the fence margin does a path avoid the fence?
 */
float AP_Avoidance_Plane::mission_avoid_fence_margin(const Location &pos1, const Location &pos2)
{
    if (num_fence_points < 4) {
        // no fence
        return _margin_fence + _margin_wide;
    }
    Vector2f diff1 = location_diff(fence_origin, pos1);
    Vector2f diff2 = location_diff(fence_origin, pos2);
    float dist = Polygon_closest_distance_line(fence_points, num_fence_points, diff1, diff2);
    if (dist <= 0) {
        // we are crossing, see by how much
        dist = -Polygon_closest_distance_point(fence_points, num_fence_points, diff2);
    }
    dist -= _margin_fence;
    return dist;
}

/*
  get the distance to intersection with the fence
 */
float AP_Avoidance_Plane::fence_distance(const Location &loc)
{
    Vector2f diff = location_diff(fence_origin, loc);
    float dist = Polygon_closest_distance_point(fence_points, num_fence_points, diff);
    if (Polygon_outside(diff, fence_points, num_fence_points)) {
        return -dist;
    }
    return dist;
}

float AP_Avoidance_Plane::calc_avoidance_margin(const Location &loc1, const Location &loc2, const Vector2f &our_velocity, float avoid_sec)
{
    float ex_margin = mission_exclusion_margin(loc1, loc2);
    float obs_margin = mission_avoidance_margin(loc1, our_velocity, avoid_sec);
    float fence_margin = mission_avoid_fence_margin(loc1, loc2);
    return MIN(MIN(ex_margin, obs_margin), fence_margin);
}


/*
  update waypoint to avoid dynamic obstacles. Return true if doing avoidance
 */
bool AP_Avoidance_Plane::update_mission_avoidance(const Location &current_loc, Location &target_loc, float groundspeed)
{
    current_lookahead = constrain_float(current_lookahead, _lookahead*0.5, _lookahead);
    const float full_distance = get_distance(current_loc, target_loc);
    /*
      the distance we look ahead is adjusted dynamically based on avoidance results
     */
    const float avoid_step1_m = current_lookahead;
    const float avoid_step2_m = current_lookahead*2;
    // we test for flying past the waypoint, so if we are close, we
    // have room to dodge after the waypoint
    const float avoid_max = MIN(avoid_step1_m, full_distance+MIN(_margin_fence/2,100));
    const float avoid_sec1 = avoid_max / groundspeed;
    const int32_t bearing_inc_cd = 500;
    const float distance = groundspeed * avoid_sec1;
    const int32_t bearing_cd = get_bearing_cd(current_loc, target_loc);

    // report collisions
    have_collided(current_loc);

    if (num_fence_points >= 4) {
        float distance_to_fence = fence_distance(current_loc);
        if (distance_to_fence < _margin_fence || (fence_avoidance && distance_to_fence < _margin_fence*1.2)) {
            /*
              we have come within the fence margin of the fence. Continue
              flying away from the fence until we are over 1.2 times the
              margin away
            */
            fence_best_avoidance(current_loc, target_loc);
            if (!fence_avoidance) {
                debug(2,"Fence avoidance %.0f/%.0f", distance_to_fence, _margin_fence);
            }
            fence_avoidance = true;
            gcs_action = (MAV_COLLISION_ACTION)4;
            gcs_threat.src_id = 0;
            gcs_threat.threat_level = MAV_COLLISION_THREAT_LEVEL_HIGH;
            gcs_threat.time_to_closest_approach = 0;
            gcs_threat.closest_approach_xy = distance_to_fence;
            gcs_threat.closest_approach_z = 0;
            return true;
        }
    }
    fence_avoidance = false;

    if (full_distance < 20) {
        // if we are within 20m, go for it
        return false;
    }

    // load exclusion zones from the mission, if any
    load_exclusion_zones();

    // load inner fence, if any
    load_fence_boundary();

    // try all 5 degree increments around a circle, alternating left
    // and right. For each one check that if we flew in that direction
    // that we would avoid all obstacles
    float best_bearing = bearing_cd*0.01;
    bool have_best_bearing = false;
    float best_margin = -10000;
    int32_t best_margin_bearing = best_bearing;
    const float rate_of_turn_dps = degrees(GRAVITY_MSS * tanf(radians(plane.aparm.roll_limit_cd*0.01*0.6))/(groundspeed+0.1));
    
    // get our ground course
    float ground_course_deg;
    {
        WITH_SEMAPHORE(plane.ahrs.get_semaphore());
        ground_course_deg = wrap_180(degrees(plane.ahrs.groundspeed_vector().angle()));
    }

    for (uint8_t i=0; i<360 / (bearing_inc_cd/100); i++) {
        int32_t bearing_delta_cd = i*bearing_inc_cd/2;
        if (i & 1) {
            // we alternate between trying to the left and right of the target
            bearing_delta_cd = -bearing_delta_cd;
        }
        // bearing that we are probing
        const int32_t bearing_test_cd = wrap_180_cd(bearing_cd + bearing_delta_cd);
        const float bearing_test = bearing_test_cd * 0.01;
        const float course_change_deg = wrap_180(bearing_test - ground_course_deg);

        if (fabsf(course_change_deg) > 170) {
            // don't consider 180 degree turns, as we can't predict which way we will turn
            continue;
        }

        // work out how long it will take to change course
        float turn_time_s = fabsf(course_change_deg / rate_of_turn_dps);

        // approximate a turn by flying forward for turn_time_s/2
        Location projected_loc = location_project(current_loc, ground_course_deg, groundspeed * turn_time_s*0.5);

        // if we're turning by more than 90 degrees then add some sideways movement
        if (fabsf(course_change_deg) > 90) {
            float direction = course_change_deg>0?(ground_course_deg+90):(ground_course_deg-90);
            float proportion = sinf(radians(fabsf(course_change_deg) - 90));
            projected_loc = location_project(projected_loc, direction, groundspeed * proportion * turn_time_s*0.5);
        }
        
        // position we will get to
        Location loc_test = location_project(projected_loc, bearing_test, distance);

        // difference from current_loc as floats
        Vector2f loc_diff = location_diff(projected_loc, loc_test);
        Vector2f our_velocity = loc_diff / avoid_sec1;

        float margin = calc_avoidance_margin(projected_loc, loc_test, our_velocity, avoid_sec1);
        if (margin > best_margin) {
            best_margin_bearing = bearing_test;
            best_margin = margin;
        }
        if (margin > _margin_wide) {
            // This bearing will avoid all dynamic and static
            // obstacles for one step of 'distance'. Now check if it
            // will put us in a position where we have a clear path to
            // our destination for distance*avoid_ratio
            if (!have_best_bearing) {
                best_bearing = bearing_test;
                have_best_bearing = true;
            } else if (fabsf(wrap_180(ground_course_deg - bearing_test)) <
                       fabsf(wrap_180(ground_course_deg - best_bearing))) {
                // replace bearing with one that is closer to our current ground course
                debug(3, "replace %.1f with %.1f, gc=%.1f\n", best_bearing, bearing_test, ground_course_deg);
                best_bearing = bearing_test;
            }

            const float test_bearings[] = { 0, 45, -45 };
            const float target_bearing = get_bearing_deg(loc_test, target_loc);
            for (uint8_t j=0; j<ARRAY_SIZE(test_bearings); j++) {
                float new_bearing = target_bearing + test_bearings[j];
                float target_distance2 = get_distance(loc_test, target_loc);
                float distance2 = constrain_float(avoid_step2_m, 10, target_distance2);
                float avoid_sec2 = distance2 / groundspeed;
                Location loc_test2 = location_project(loc_test, new_bearing, distance2);

                Vector2f loc_diff2 = location_diff(loc_test, loc_test2);
                Vector2f our_velocity2 = loc_diff2 / avoid_sec2;

                float margin2 = calc_avoidance_margin(loc_test, loc_test2, our_velocity2, avoid_sec2);
                if (margin2 > _margin_wide) {
                    // all good, now project in the chosen direction by the full distance
                    target_loc = location_project(projected_loc, bearing_test, full_distance);
                    debug(4,"good: i=%d j=%d bt:%d nb:%d m1:%.1f m2:%.1f\n",
                          i, j, int(bearing_test), int(new_bearing), margin, margin2);
                    current_lookahead = MIN(_lookahead, current_lookahead*1.1);
                    log_avoidance(0, bearing_delta_cd*0.01, margin, margin2);
                    bool ret = (i != 0 || j != 0);
                    if (!ret) {
                        gcs_action = (MAV_COLLISION_ACTION)0;
                    } else {
                        gcs_action = (MAV_COLLISION_ACTION)1;
                    }
                    return ret;
                }
            }
        }
    }

    float chosen_bearing;
    if (have_best_bearing) {
        // none of the directions tested were OK for 2-step checks. Choose the direction
        // that was best for the first step
        debug(2, "bad1: bb=%d bm:%.1f\n", int(best_bearing), best_margin);
        chosen_bearing = best_bearing;
        current_lookahead = MIN(_lookahead, current_lookahead*1.05);
        gcs_action = (MAV_COLLISION_ACTION)2;
        log_avoidance(1, wrap_180(chosen_bearing - (bearing_cd*0.01)), best_margin, -1);
    } else {
        // none of the possible paths had a positive margin. Choose
        // the one with the highest margin
        debug(2,"bad2: bmb=%d bm:%.1f\n", int(best_margin_bearing), best_margin);
        chosen_bearing = best_margin_bearing;
        current_lookahead = MAX(_lookahead*0.5, current_lookahead*0.9);
        gcs_action = (MAV_COLLISION_ACTION)3;
        log_avoidance(2, wrap_180(chosen_bearing - (bearing_cd*0.01)), best_margin, -1);
    }

    // calculate new target based on best effort
    target_loc = location_project(current_loc, chosen_bearing, full_distance);


    return true;
}

/*
  find the target_loc which best avoids the fence
 */
void AP_Avoidance_Plane::fence_best_avoidance(const Location &current_loc, Location &target_loc)
{
    const int32_t bearing_inc = 5;
    float best_dist = -10000;
    for (uint8_t i=0; i<360 / bearing_inc; i++) {
        float bearing = i * bearing_inc;
        Location test_loc = location_project(current_loc, bearing, _margin_fence*0.5);
        Vector2f diff = location_diff(fence_origin, test_loc);
        float fence_dist = Polygon_closest_distance_point(fence_points, num_fence_points, diff);
        if (Polygon_outside(diff, fence_points, num_fence_points)) {
            fence_dist = -5000 - fence_dist;
        }
        if (fence_dist > best_dist) {
            target_loc = test_loc;
            best_dist = fence_dist;
        }
    }
}

/*
  avoidance thread. This continually updates the avoidance_result
  structure based on avoidance_request
 */
void AP_Avoidance_Plane::avoidance_thread(void)
{
    while (true) {
        hal.scheduler->delay(100);
        Location current_loc;
        Location target_loc;
        Location new_target_loc;
        float groundspeed;
        {
            WITH_SEMAPHORE(_rsem);
            uint32_t now = AP_HAL::millis();
            if (now - avoidance_request.request_time_ms > 2000) {
                // this is a very old request, don't process it
                continue;
            }
            current_loc = avoidance_request.current_loc;
            target_loc = new_target_loc = avoidance_request.target_loc;
            groundspeed = avoidance_request.groundspeed;
        }

        bool res = update_mission_avoidance(current_loc, new_target_loc, groundspeed);
        {
            // give the main thread the avoidance result
            WITH_SEMAPHORE(_rsem);
            avoidance_result.target_loc = target_loc;
            avoidance_result.new_target_loc = res?new_target_loc:target_loc;
            avoidance_result.result_time_ms = AP_HAL::millis();
            avoidance_result.avoidance_needed = res;
        }
    }
}

/*
  update avoidance request for the thread to work on, and see if there
  is a result that matches the current target
 */
bool AP_Avoidance_Plane::mission_avoidance(const Location &current_loc, Location &target_loc, float groundspeed)
{
    if (!_enabled || _warn_action != 2) {
        return false;
    }

    WITH_SEMAPHORE(_rsem);

    if (!thread_created) {
        // create the avoidance thread as low priority. It should soak
        // up spare CPU cycles to fill in the avoidance_result structure based
        // on requests in avoidance_request
        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Avoidance_Plane::avoidance_thread, void),
                                          "avoidance",
                                          4096, AP_HAL::Scheduler::PRIORITY_IO, -1)) {
            return false;
        }
        thread_created = true;
    }

    uint32_t now = AP_HAL::millis();

    // place new request for the thread to work on
    avoidance_request.current_loc = current_loc;
    avoidance_request.target_loc = target_loc;
    avoidance_request.groundspeed = groundspeed;
    avoidance_request.request_time_ms = now;

    if (target_loc.lat == avoidance_result.target_loc.lat &&
        target_loc.lng == avoidance_result.target_loc.lng &&
        now - avoidance_result.result_time_ms < 2000) {
        // we have a result from the thread
        target_loc = avoidance_result.new_target_loc;
        return avoidance_result.avoidance_needed;
    }

    return false;
}

void AP_Avoidance_Plane::log_avoidance(uint8_t result, float bearing_change, float margin1, float margin2)
{
    DataFlash_Class::instance()->Log_Write("AVDM", "TimeUS,Res,BCh,M1,M2", "QBfff",
                                           AP_HAL::micros64(),
                                           result, bearing_change, margin1, margin2);
}

/*
  return the current threat
 */
AP_Avoidance::Obstacle *AP_Avoidance_Plane::most_serious_threat()
{
    return &gcs_threat;
}

/*
  return current avoidance action
 */
MAV_COLLISION_ACTION AP_Avoidance_Plane::mav_avoidance_action()
{
    if (collision_detected) {
        return (MAV_COLLISION_ACTION)5;
    }
    return gcs_action;
}

/*
  get height difference between an obstacle and our location
 */
float AP_Avoidance_Plane::obstacle_height_difference(const Obstacle &obstacle)
{
    int32_t alt1_cm=0;
    const Location_Class &loc = obstacle._location;
    loc.get_alt_cm(Location_Class::ALT_FRAME_ABSOLUTE, alt1_cm);

    int32_t alt2_cm=0;
    const Location_Class &myloc = plane.current_loc;
    myloc.get_alt_cm(Location_Class::ALT_FRAME_ABSOLUTE, alt2_cm);
    return labs(alt1_cm - alt2_cm) * 0.01;
}
