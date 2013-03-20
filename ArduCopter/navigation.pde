// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// update_navigation - invokes navigation routines
// called at 10hz
static void update_navigation()
{
    static uint32_t nav_last_update = 0;        // the system time of the last time nav was run update

    // check for inertial nav updates
    if( inertial_nav.position_ok() ) {

        // calculate time since nav controllers last ran
        dTnav = (float)(millis() - nav_last_update)/ 1000.0f;
        nav_last_update = millis();

        // prevent runnup in dTnav value
        dTnav = min(dTnav, 1.0f);

        // run the navigation controllers
        update_nav_mode();

        // update log
        if (g.log_bitmask & MASK_LOG_NTUN && motors.armed()) {
            Log_Write_Nav_Tuning();
        }
    }
}

// run_nav_updates - top level call for the autopilot
// ensures calculations such as "distance to waypoint" are calculated before autopilot makes decisions
// To-Do - rename and move this function to make it's purpose more clear
static void run_nav_updates(void)
{
    // fetch position from inertial navigation
    calc_position();

    // check altitude vs target
    verify_altitude();

    // calculate distance and bearing for reporting and autopilot decisions
    calc_distance_and_bearing();

    // run autopilot to make high level decisions about control modes
    run_autopilot();
}

// calc_position - get lat and lon positions from inertial nav library
static void calc_position(){
    if( inertial_nav.position_ok() ) {
        // pull position from interial nav library
        current_loc.lng = inertial_nav.get_longitude();
        current_loc.lat = inertial_nav.get_latitude();
    }
}

// calc_distance_and_bearing - calculate distance and direction to waypoints for reporting and autopilot decisions
static void calc_distance_and_bearing()
{
    Vector3f curr = inertial_nav.get_position();
    
    // get target from loiter or wpinav controller
    if( nav_mode == NAV_LOITER || nav_mode == NAV_CIRCLE ) {
        wp_distance = wp_nav.get_distance_to_target();
        wp_bearing = wp_nav.get_bearing_to_target();
    }else if( nav_mode == NAV_WP ) {
        wp_distance = wp_nav.get_distance_to_destination();
        wp_bearing = wp_nav.get_bearing_to_destination();
    }else{
        wp_distance = 0;
        wp_bearing = 0;
    }

    // calculate home distance and bearing
    if( ap.home_is_set ) {
        home_distance = pythagorous2(curr.x, curr.y);
        home_bearing = pv_get_bearing_cd(curr,Vector3f(0,0,0));

        // update super simple bearing (if required) because it relies on home_bearing
        update_super_simple_bearing();
    }else{
        home_distance = 0;
        home_bearing = 0;
    }

    // calculate bearing to target (used when yaw_mode = YAW_LOOK_AT_LOCATION)
    // To-Do: move this to the look-at-waypoint yaw controller
    if( yaw_mode == YAW_LOOK_AT_LOCATION ) {
        yaw_look_at_WP_bearing = pv_get_bearing_cd(curr, yaw_look_at_WP);
    }
}

// run_autopilot - highest level call to process mission commands
static void run_autopilot()
{
    switch( control_mode ) {
        case AUTO:
            // majority of command logic is in commands_logic.pde
            verify_commands();
            break;
        case GUIDED:
            // switch to loiter once we've reached the target location and altitude
            // To-Do: this incorrectly checks verify_nav_wp even though the nav mode may be NAV_LOITER
            if(verify_nav_wp()) {
                set_nav_mode(NAV_LOITER);
            }
        case RTL:
            verify_RTL();
            break;
    }
}

// set_nav_mode - update nav mode and initialise any variables as required
static bool set_nav_mode(uint8_t new_nav_mode)
{
    // boolean to ensure proper initialisation of nav modes
    bool nav_initialised = false;

    // return immediately if no change
    if( new_nav_mode == nav_mode ) {
        return true;
    }

    switch( new_nav_mode ) {

        case NAV_NONE:
            nav_initialised = true;
            break;

        case NAV_CIRCLE:
            // set center of circle to current position
            circle_set_center(inertial_nav.get_position(), ahrs.yaw);
            nav_initialised = true;
            break;

        case NAV_LOITER:
            // set target to current position
            wp_nav.set_loiter_target(inertial_nav.get_position());
            nav_initialised = true;
            break;

        case NAV_WP:
            nav_initialised = true;
            break;
    }

    // if initialisation has been successful update the yaw mode
    if( nav_initialised ) {
        nav_mode = new_nav_mode;
    }

    // return success or failure
    return nav_initialised;
}

// update_nav_mode - run navigation controller based on nav_mode
static void update_nav_mode()
{
    switch( nav_mode ) {

        case NAV_NONE:
            // do nothing
            break;

        case NAV_CIRCLE:
            // call circle controller which in turn calls loiter controller
            update_circle(dTnav);
            break;

        case NAV_LOITER:
            // call loiter controller
            wp_nav.update_loiter();
            break;

        case NAV_WP:
            // call waypoint controller
            wp_nav.update_wpnav();
            break;
    }

    /*
    // To-Do: check that we haven't broken toy mode
    case TOY_A:
    case TOY_M:
        set_nav_mode(NAV_NONE);
        update_nav_wp();
        break;
    }
    */
}

static bool check_missed_wp()
{
    int32_t temp;
    temp = wp_bearing - original_wp_bearing;
    temp = wrap_180_cd(temp);
    return (labs(temp) > 9000);         // we passed the waypoint by 90 degrees
}

static void force_new_altitude(int32_t new_alt)
{
    next_WP.alt     = new_alt;
    set_alt_change(REACHED_ALT);
}

static void set_new_altitude(int32_t new_alt)
{
    // if no change exit immediately
    if(new_alt == next_WP.alt) {
        return;
    }

    // update new target altitude
    next_WP.alt     = new_alt;

    if(next_WP.alt > (current_loc.alt + 80)) {
        // we are below, going up
        set_alt_change(ASCENDING);

    }else if(next_WP.alt < (current_loc.alt - 80)) {
        // we are above, going down
        set_alt_change(DESCENDING);

    }else{
        // No Change
        set_alt_change(REACHED_ALT);
    }
}

// verify_altitude - check if we have reached the target altitude
static void verify_altitude()
{
    if(alt_change_flag == ASCENDING) {
        // we are below, going up
        if(current_loc.alt >  next_WP.alt - 50) {
        	set_alt_change(REACHED_ALT);
        }
    }else if (alt_change_flag == DESCENDING) {
        // we are above, going down
        if(current_loc.alt <=  next_WP.alt + 50){
        	set_alt_change(REACHED_ALT);
        }
    }
}

// Keeps old data out of our calculation / logs
static void reset_nav_params(void)
{
    // Will be set by new command
    wp_bearing                      = 0;

    // Will be set by new command
    wp_distance                     = 0;

    // Will be set by nav or loiter controllers
    lon_error                       = 0;
    lat_error                       = 0;
    nav_roll 						= 0;
    nav_pitch 						= 0;
}

// get_yaw_slew - reduces rate of change of yaw to a maximum
// assumes it is called at 100hz so centi-degrees and update rate cancel each other out
static int32_t get_yaw_slew(int32_t current_yaw, int32_t desired_yaw, int16_t deg_per_sec)
{
    return wrap_360_cd(current_yaw + constrain(wrap_180_cd(desired_yaw - current_yaw), -deg_per_sec, deg_per_sec));
}

// valid_waypoint - checks if a waypoint has been initialised or not
static bool waypoint_valid(Location &wp)
{
     if( wp.lat != 0 || wp.lng != 0 ) {
         return true;
     }else{
         return false;
     }
}


//////////////////////////////////////////////////////////
// circle navigation controller
//////////////////////////////////////////////////////////

// circle_set_center -- set circle controller's center position and starting angle
static void
circle_set_center(const Vector3f pos_vec, float heading_in_radians)
{
    // set circle center
    circle_center = pos_vec;

    // set starting angle to current heading - 180 degrees
    circle_angle = heading_in_radians-ToRad(180);
    if( circle_angle > 180 ) {
        circle_angle -= 180;
    }
    if( circle_angle < -180 ) {
        circle_angle -= 180;
    }

    // initialise other variables
    circle_angle_total = 0;
}

// circle_get_pos - circle position controller's main call which in turn calls loiter controller with updated target position
static void
update_circle(float dt)
{
    float angle_delta = circle_rate * dt;
    float cir_radius = g.circle_radius * 100;
    Vector3f circle_target;

    // update the target angle
    circle_angle += angle_delta;
    if( circle_angle > 180 ) {
        circle_angle -= 360;
    }
    if( circle_angle <= -180 ) {
        circle_angle += 360;
    }

    // update the total angle travelled
    circle_angle_total += angle_delta;

    // calculate target position
    circle_target.x = circle_center.x + cir_radius * sinf(1.57f - circle_angle);
    circle_target.y = circle_center.y + cir_radius * cosf(1.57f - circle_angle);

    // re-use loiter position controller
    wp_nav.set_loiter_target(circle_target);

    // call loiter controller
    wp_nav.update_loiter();
}
