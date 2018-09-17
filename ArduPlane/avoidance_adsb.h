#pragma once

#include <AP_Avoidance/AP_Avoidance.h>

// Provide Plane-specific implementation of avoidance.  While most of
// the logic for doing the actual avoidance is present in
// AP_Avoidance, this class allows Plane to override base
// functionality - for example, not doing anything while landed.
class AP_Avoidance_Plane : public AP_Avoidance {
public:
    AP_Avoidance_Plane(AP_AHRS &ahrs, class AP_ADSB &adsb)
        : AP_Avoidance(ahrs, adsb)
    {
    }

    /* Do not allow copies */
    AP_Avoidance_Plane(const AP_Avoidance_Plane &other) = delete;
    AP_Avoidance_Plane &operator=(const AP_Avoidance_Plane&) = delete;

    bool mission_avoidance(const Location &loc, Location &target_loc, float groundspeed);

    // check if we are clear to a good radius for takeoff/landing
    bool mission_clear(const Location &current_loc, float xy_clearance, float z_clearance, float time_s);

    bool mission_avoidance_enabled(void) const {
        return _enabled && _warn_action == 2;
    }

protected:
    // override avoidance handler
    MAV_COLLISION_ACTION handle_avoidance(const AP_Avoidance::Obstacle *obstacle, MAV_COLLISION_ACTION requested_action) override;

    // override recovery handler
    void handle_recovery(uint8_t recovery_action) override;

    // check flight mode is avoid_adsb
    bool check_flightmode(bool allow_mode_change);

    // vertical avoidance handler
    bool handle_avoidance_vertical(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change);

    // horizontal avoidance handler
    bool handle_avoidance_horizontal(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change);

    Obstacle *most_serious_threat(void) override;
    MAV_COLLISION_ACTION mav_avoidance_action() override;

    // control mode before avoidance began
    FlightMode prev_control_mode = RTL;

private:
    float mission_avoidance_margin(const Location &our_loc, const Vector2f &our_velocity, float avoid_sec);
    float mission_exclusion_margin(const Location &current_loc, const Location &loc_test);
    float mission_avoid_fence_margin(const Location &pos1, const Location &pos2);
    void load_exclusion_zones(void);
    void unload_exclusion_zones(void);
    void load_fence_boundary(void);
    float get_avoidance_radius(const class Obstacle &obstacle) const;
    bool within_avoidance_height(const class Obstacle &obstacle, const float margin, float dt) const;
    bool have_collided(const Location &loc);
    void fence_best_avoidance(const Location &current_loc, Location &target_loc, float ground_course_deg);
    float predict_closest_fence_distance(const Location &current_loc, float bank_target, float ground_course_deg, float t);
    float fence_distance(const Location &current_loc);
    float calc_avoidance_margin(const Location &loc1, const Location &loc2, const Vector2f &our_velocity, float avoid_sec);
    void log_avoidance(uint8_t result, float bearing_change, float margin1, float margin2);
    float obstacle_height_difference(const Obstacle &obstacle);

    bool thread_created;
    void avoidance_thread(void);

    // an avoidance request from the navigation code
    struct avoidance_info {
        Location current_loc;
        Location target_loc;
        float groundspeed;
        float airspeed;
        float wind_dir_rad;
        float wind_speed;
        uint32_t request_time_ms;
    } avoidance_request, avoid_req2;

    bool update_mission_avoidance(const avoidance_info &avd, Location &target_loc);
    float effective_groundspeed(float airspeed, float bearing_deg, float wind_dir_rad, float wind_speed);

    // an avoidance result from the avoidance thread
    struct {
        Location target_loc;
        Location new_target_loc;
        uint32_t result_time_ms;
        bool avoidance_needed;
    } avoidance_result;

    uint8_t num_exclusion_zones;
    struct exclusion_zone {
        // closed polygon, with points[0] == points[n-1]
        uint8_t num_points;
        Vector2f *points;
        // lat/lon of first point. Other points are relative to that
        // point
        Location first_loc;
        uint16_t first_wp;
    } *exclusion_zones;
    uint32_t mission_change_ms;

    // georefence as floats with origin
    uint8_t num_fence_points;
    Location fence_origin;
    Vector2f *fence_points;
    uint32_t last_fence_change_ms;
    bool fence_avoidance;
    float current_lookahead;

    Obstacle gcs_threat;
    MAV_COLLISION_ACTION gcs_action;
    bool collision_detected;

    enum {
        OPTION_IGNORE_HEIGHT=1<<0,
    };
};
