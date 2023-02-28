#include <AP_Math/AP_Math.h>
#include "AC_Adv_Failsafe.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

const AP_Param::GroupInfo AC_Adv_Failsafe::var_info[] = {
    
    // User configurable
    AP_GROUPINFO("_ENA",       0, AC_Adv_Failsafe, _failsafe_int_point_enabled,      0),

    AP_GROUPINFO("_ALT_CM",    1, AC_Adv_Failsafe, _failsafe_travel_altitude,        1500), // in cm

    AP_GROUPINFO("_MAXDST_M",  2, AC_Adv_Failsafe, _failsafe_max_distance_from_home, 500), // in meters

    // Non Writable, only to persist reboot, it is the actual state of the mission interrupted
    AP_GROUPINFO("_NW_LAT",    3, AC_Adv_Failsafe, _failsafe_latitude,                FS_DEFAULTS),

    AP_GROUPINFO("_NW_LON",    4, AC_Adv_Failsafe, _failsafe_longitude,               FS_DEFAULTS),

    AP_GROUPINFO("_NW_ALT",    5, AC_Adv_Failsafe, _failsafe_altitude,                FS_DEFAULTS),

    AP_GROUPINFO("_NW_ACT",    6, AC_Adv_Failsafe, _failsafe_active,                  FS_DEFAULTS),

    AP_GROUPINFO("_NW_WP",     7, AC_Adv_Failsafe, _failsafe_wp_number,               FS_DEFAULTS),

    AP_GROUPINFO("_NW_VEL",    8, AC_Adv_Failsafe, _failsafe_velocity,                FS_DEFAULTS), // in cm/s

    AP_GROUPINFO("_NW_TDIST",  9, AC_Adv_Failsafe, _failsafe_trigger_dist,            FS_DEFAULTS), // in cm/s

    AP_GROUPEND
};

AC_Adv_Failsafe::AC_Adv_Failsafe() 
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("Mount must be singleton");
    }
    _singleton = this;
}

// On init se set the stage to takeoff if the parameter of active failsafe is set.
// we need to do this here as in the constructor the parameter system isn't yet initialized
void AC_Adv_Failsafe::init() 
{
    // If failsafe active ( ADFS_NW_ACT = 1 ) perform sanity checks, and disable if not successfull
    if (_failsafe_active) {
        // Sanity checks
        bool good_to_go = true;

        if (_failsafe_latitude == 0) {
            good_to_go = false;
        }
        if (_failsafe_longitude == 0) {
            good_to_go = false;
        }
        if (_failsafe_altitude == 0) {
            good_to_go = false;
        }
        if (_failsafe_wp_number == 0) {
            good_to_go = false;
        }

        if (!good_to_go) {
            _failsafe_active.set_and_save(0);
            _failsafe_trigger_dist.set_and_save(0);
            gcs().send_text(MAV_SEVERITY_WARNING, "Return to mission data corrupted, resetting state");
            return;
        }

        // if we made it until here we should be good to go, set stage to take off
        _fs_stage = StartTakeOff;
        // gcs().send_text(MAV_SEVERITY_INFO, "ADV FAILSAFE STAGE SET TO TAKEOFF");
    } else {
        // If no failsafe active, clear everything, just in case
        clear_failsafe_status();
    }
}

MAV_RESULT AC_Adv_Failsafe::handle_send_fs_coords_to_gcs()
{
    mavlink_named_value_int_t packetnew {};
    packetnew.time_boot_ms = AP_HAL::millis();
    
    char name_lat[10] = "FS_LAT";
    packetnew.value = _failsafe_latitude;
    memcpy(packetnew.name, name_lat, MIN(strlen(name_lat), (uint8_t)MAVLINK_MSG_NAMED_VALUE_INT_FIELD_NAME_LEN));
    gcs().send_to_active_channels(MAVLINK_MSG_ID_NAMED_VALUE_INT,
                            (const char *)&packetnew);
                            
    char name_long[10] = "FS_LON";
    packetnew.value = _failsafe_longitude;
    memcpy(packetnew.name, name_long, MIN(strlen(name_long), (uint8_t)MAVLINK_MSG_NAMED_VALUE_INT_FIELD_NAME_LEN));
    gcs().send_to_active_channels(MAVLINK_MSG_ID_NAMED_VALUE_INT,
                            (const char *)&packetnew);
    
    return MAV_RESULT_ACCEPTED;
}

void AC_Adv_Failsafe::save_failsafe_status(int32_t latitude, int32_t longitude, int32_t altitude, uint16_t wp_number, float velocity, float tdist)
{
    _failsafe_latitude.set_and_save(latitude);
    _failsafe_longitude.set_and_save(longitude);
    _failsafe_altitude.set_and_save(altitude);
    _failsafe_wp_number.set_and_save(wp_number);
    _failsafe_velocity.set_and_save(velocity);
    _failsafe_trigger_dist.set_and_save(tdist);

    _fs_stage = StartTakeOff;
}

void AC_Adv_Failsafe::clear_failsafe_status()
{
    _failsafe_latitude.set_and_save(0);
    _failsafe_longitude.set_and_save(0);
    _failsafe_altitude.set_and_save(0);
    _failsafe_active.set_and_save(0);
    _failsafe_wp_number.set_and_save(0);
    _failsafe_velocity.set_and_save(0);
    _failsafe_trigger_dist.set_and_save(0);

    _fs_stage = NotActive;

    gcs().send_text(MAV_SEVERITY_INFO, "Continue Mission Coords cleared");
}

void AC_Adv_Failsafe::log_failsafe_status()
{
    // missing enable parameter
    AP::logger().Write("ADFS", "TimeUS,lat,lon,alt,vel,tdi", "Qiiiff",
        AP_HAL::micros64(),
        _failsafe_latitude,
        _failsafe_longitude,
        _failsafe_altitude,
        _failsafe_velocity,
        _failsafe_trigger_dist);
}

void AC_Adv_Failsafe::set_active(bool active)
{
    int setactive = 0;
 
    if (active) {
        // perform this sanity check of variables non-zero to prevent problems in all manual flights with RTL or Land at the end
        if ((_failsafe_latitude != 0) && (_failsafe_longitude != 0) && (_failsafe_altitude != 0) && (_failsafe_wp_number != 0))
            setactive = 1;    
    }
    _failsafe_active.set_and_save(setactive);
}

// singleton instance
AC_Adv_Failsafe *AC_Adv_Failsafe::_singleton;

namespace AP {

AC_Adv_Failsafe *ac_adv_failsafe()
{
    return AC_Adv_Failsafe::get_singleton();
}

};