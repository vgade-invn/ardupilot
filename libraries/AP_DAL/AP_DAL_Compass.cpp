#include "AP_DAL_Compass.h"

#include <AP_Compass/AP_Compass.h>

#include <AP_Logger/AP_Logger.h>

AP_DAL_Compass::AP_DAL_Compass()
{
    for (uint8_t i=0; i<ARRAY_SIZE(_RMGI); i++) {
        _RMGI[i].instance = i;
    }
}

void AP_DAL_Compass::start_frame(const uint64_t time_us)
{
    const auto &compass = AP::compass();

    const log_RMGH old = _RMGH;
    _RMGH.count = compass.get_count();
    _RMGH.auto_declination_enabled = compass.auto_declination_enabled();
    _RMGH.declination = compass.get_declination();
    _RMGH.num_enabled = compass.get_num_enabled();
    _RMGH.consistent = compass.consistent();

    if (STRUCT_NEQ(old, _RMGH)) {
        WRITE_REPLAY_BLOCK(RMGH, _RMGH);
    }

    for (uint8_t i=0; i<ARRAY_SIZE(_RMGI); i++) {
        log_RMGI &RMGI = _RMGI[i];
        const log_RMGI old_RMGI = RMGI;
        const uint32_t last_update_usec = compass.last_update_usec(i);
        if (last_update_usec == _last_logged_update_usec[i]) {
            continue;
        }
        _last_logged_update_usec[i] = last_update_usec;
        RMGI.use_for_yaw = compass.use_for_yaw(i);
        RMGI.healthy = compass.healthy(i);
        memcpy((void*)&RMGI.offsets, (void*)&compass.get_offsets(i), sizeof(Vector3f));
        RMGI.have_scale_factor = compass.have_scale_factor(i);
        RMGI.last_update_usec = last_update_usec;
        memcpy((void*)&RMGI.field, (void*)&compass.get_field(i), sizeof(Vector3f));

        if (STRUCT_NEQ(old_RMGI, _RMGI)) {
            WRITE_REPLAY_BLOCK(RMGI, RMGI);
        }
    }
}
