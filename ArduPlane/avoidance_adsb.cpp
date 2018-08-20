
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
  given our location, velocity and a avoidance time, find the closest
  we will come to any of the obstacles avoidance radiuses
  A negative result means we will come within the avoidance radius of at least one object
 */
float AP_Avoidance_Plane::mission_avoidance_margin(const Location &our_loc, const Vector2f &our_velocity, float avoid_sec)
{
    uint32_t now = AP_HAL::millis();
    const uint32_t timeout_ms = 5000;
    float margin = avoidance_large_m;
    
    for (uint8_t i=0; i<_obstacle_count; i++) {
        const Obstacle &obstacle = _obstacles[i];
        if (now - obstacle.timestamp_ms > timeout_ms) {
            continue;
        }
        if (!within_avoidance_height(obstacle)) {
            continue;
        }

        // use our starting point as origin
        const Vector2f obstacle_position = location_diff(our_loc, obstacle._location);
        // get our velocity relative to obstacle
        const Vector2f relative_velocity = our_velocity - Vector2f(obstacle._velocity.x,obstacle._velocity.y);
        const Vector2f final_pos = relative_velocity * avoid_sec;

        // lookup the min distance to keep from the object
        const float radius = get_avoidance_radius(obstacle);

        float dist = Vector2f::closest_distance_between_radial_and_point(final_pos, obstacle_position) - radius;
        if (dist < margin) {
            margin = dist;
        }
    }

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
    float margin = avoidance_large_m;
    for (uint8_t zone=0; zone<num_exclusion_zones; zone++) {
        const struct exclusion_zone &ezone = exclusion_zones[zone];
        const Vector2f p1 = location_diff(ezone.first_loc, current_loc);
        const Vector2f p2 = location_diff(ezone.first_loc, loc_test);
        float dist = Polygon_closest_distance(ezone.points, ezone.num_points, p1, p2) - exclusion_margin_static;
        if (dist < margin) {
            margin = dist;
        }
    }
    return margin;
}


/*
  check if we have already collided with a dynamic obstacle
 */
bool AP_Avoidance_Plane::have_collided(const Location &current_loc)
{
    uint32_t now = AP_HAL::millis();
    const uint32_t timeout_ms = 5000;
    
    for (uint8_t i=0; i<_obstacle_count; i++) {
        const Obstacle &obstacle = _obstacles[i];
        if (now - obstacle.timestamp_ms > timeout_ms) {
            continue;
        }
        if (!within_avoidance_height(obstacle)) {
            continue;
        }
        const float radius = get_avoidance_radius(obstacle);
        float distance = get_distance(current_loc, obstacle._location);
        if (distance < radius) {
            //::printf("Collided with %u at %.0fm\n", obstacle.src_id, distance);
            return true;
        }
    }
    return false;
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
        loc = location_project(center_loc, bearing, MAX(distance+change_m, 1));
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

    // copy the fence, so we can expand it
    fence_points = new Vector2l[num_points];
    if (fence_points == nullptr) {
        return;
    }
    memcpy(fence_points, points, num_points * sizeof(fence_points[0]));
    num_fence_points = num_points;

    grow_polygonl(fence_points, num_fence_points, -fence_margin);
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

/*
  update waypoint to avoid dynamic obstacles. Return true if doing avoidance
 */
bool AP_Avoidance_Plane::update_mission_avoidance(const Location &current_loc, Location &target_loc, float groundspeed)
{
    const float full_distance = get_distance(current_loc, target_loc);
    const float avoid_step1_m = 500;
    const float avoid_step2_m = 1000;
    const float avoid_max = MIN(avoid_step1_m, full_distance-1);
    const float avoid_sec1 = avoid_max / groundspeed;
    const int32_t bearing_inc_cd = 500;
    const float distance = groundspeed * avoid_sec1;
    const int32_t bearing_cd = get_bearing_cd(current_loc, target_loc);

    // report collisions
    have_collided(current_loc);
    
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
    int32_t best_bearing = bearing_cd*0.01;
    bool have_best_bearing = false;
    float best_margin = -10000;
    int32_t best_margin_bearing = best_bearing;

    for (uint8_t i=0; i<360 / (bearing_inc_cd/100); i++) {
        int32_t bearing_delta_cd = i*bearing_inc_cd/2;
        if (i & 1) {
            bearing_delta_cd = -bearing_delta_cd;
        }
        // bearing that we are probing
        const int32_t bearing_test_cd = wrap_180_cd(bearing_cd + bearing_delta_cd);
        const float bearing_test = bearing_test_cd * 0.01;

        // position we will get to
        Location loc_test = location_project(current_loc, bearing_test, distance);

        // difference from current_loc as floats
        Vector2f loc_diff = location_diff(current_loc, loc_test);
        Vector2f our_velocity = loc_diff / avoid_sec1;

        if (!mission_avoid_fence(loc_test)) {
            // don't go in a direction that could hit the fence
            continue;
        }
        float ex_margin = mission_exclusion_margin(current_loc, loc_test);
        float obs_margin = mission_avoidance_margin(current_loc, our_velocity, avoid_sec1);
        float margin = MIN(ex_margin, obs_margin);
        if (margin > best_margin) {
            best_margin_bearing = bearing_test;
            best_margin = margin;
        }
        if (margin > 0) {
            // This bearing will avoid all dynamic and static
            // obstacles for one step of 'distance'. Now check if it
            // will put us in a position where we have a clear path to
            // our destination for distance*avoid_ratio
            if (!have_best_bearing) {
                best_bearing = bearing_test;
                have_best_bearing = true;
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

                if (!mission_avoid_fence(loc_test2)) {
                    continue;
                }
                float ex_margin2 = mission_exclusion_margin(loc_test, loc_test2);
                float obs_margin2 = mission_avoidance_margin(loc_test, our_velocity2, avoid_sec2);
                float margin2 = MIN(ex_margin2, obs_margin2);
                if (margin2 > 0) {
                    // all good, now project in the chosen direction by the full distance
                    target_loc = location_project(current_loc, bearing_test, full_distance);
                    //::printf("good: i=%d j=%d bearing_test:%d new_bearing:%d margin1:%.1f margin2:%.1f\n",
                    //i, j, int(bearing_test), int(new_bearing), margin, margin2);
                    return i != 0 || j != 0;
                }
            }
        }
    }

    if (have_best_bearing) {
        // None of the directions had a positive margin. Go in the
        // direction with the best margin
        target_loc = location_project(current_loc, best_margin_bearing, full_distance);
        //::printf("bad1: best_margin_bearing=%d best_margin:%.1f\n",
        //         int(best_margin_bearing), best_margin);
    } else {
        // none of the possible paths were OK for our tests, choose the
        // one that did best on the first step
        target_loc = current_loc;
        location_update(target_loc, best_bearing, full_distance);
        //::printf("bad2: best_bearing=%d best_margin:%.1f\n",
        //         int(best_bearing), best_margin);
    }

    return true;
}

/*
  avoidance thread. This continually updates the avoidance_result
  structure based on avoidance_request
 */
void AP_Avoidance_Plane::avoidance_thread(void)
{
    while (true) {
        hal.scheduler->delay(50);
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
            avoidance_result.new_target_loc = new_target_loc;
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
