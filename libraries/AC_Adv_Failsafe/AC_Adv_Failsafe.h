#pragma once

#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Param/AP_Param.h>

#define FS_DEFAULTS 0
#define FS_MIN_TRAVEL_ALT 500  

class AC_Adv_Failsafe
{
public:
    // Constructor
    AC_Adv_Failsafe();

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

    // Initialization, called on copter.init()
    void init();

    // Adv return to mission stages
    enum STAGE {
        NotActive = 0,
        StartTakeOff,
        AscentToFSCoordAlt,
        GotoFSCoord,
        DescentToFSCoordAlt,
    };

    // Misc functions
    MAV_RESULT handle_send_fs_coords_to_gcs ();
    void       clear_failsafe_status        ();
    void       log_failsafe_status          ();
    void       save_failsafe_status         (int32_t latitude, int32_t longitude, int32_t altitude, 
                                             uint16_t wp_number, float velocity, float tdist);
    // Setters
    void       set_active                   (bool active); 
    void       set_stage                    (STAGE stage)  { _fs_stage = stage; }
    void       set_trigg_dist               (float tdist)  { _failsafe_trigger_dist.set_and_save(tdist); }

    // Getters
    int32_t    get_travel_altitude          () { return MAX(_failsafe_travel_altitude, FS_MIN_TRAVEL_ALT); } // Limited to FS_MIN_TRAVEL_ALT for extra safety
    bool       get_enabled                  () { return _failsafe_int_point_enabled; }
    bool       get_active                   () { return _failsafe_active; }
    STAGE      get_stage                    () { return _fs_stage; }
    int32_t    get_latitude                 () { return _failsafe_latitude; }
    int32_t    get_longitude                () { return _failsafe_longitude; }
    int32_t    get_altitude                 () { return _failsafe_altitude; }
    uint16_t   get_index                    () { return _failsafe_wp_number; }
    float      get_velocity                 () { return _failsafe_velocity; }
    float      get_trigg_dist               () { return _failsafe_trigger_dist; }
    int32_t    get_max_dist_from_home       () { return _failsafe_max_distance_from_home; }

    // get singleton instance, handy to invoke it from anywhere
    static AC_Adv_Failsafe *get_singleton() { return _singleton; }

private:

    // These are the non write parameters, used only to persist reboots
    AP_Int32   _failsafe_latitude;          // Latitude saved
    AP_Int32   _failsafe_longitude;         // Longitude saved
    AP_Int32   _failsafe_altitude;          // Altitude saved
    AP_Int8    _failsafe_active;            // True when we should go to failsafe point, false otherwise
    AP_Int16   _failsafe_wp_number;
    AP_Float   _failsafe_velocity;
    AP_Float   _failsafe_trigger_dist;

    // These are writable parameters for controlling behaviour
    AP_Int8    _failsafe_int_point_enabled;
    AP_Int16   _failsafe_travel_altitude; // this is the altitude at wich the copter will go to the continue mission point
    AP_Int32   _failsafe_max_distance_from_home;

    // Stage used on guided mode
    STAGE      _fs_stage = NotActive;

    // Singleton, handy to invoke it from anywhere
    static AC_Adv_Failsafe *_singleton;
};

namespace AP {
    AC_Adv_Failsafe *ac_adv_failsafe();
};