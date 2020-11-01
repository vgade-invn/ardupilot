#include "AP_DAL_Beacon.h"

#include <AP_Beacon/AP_Beacon.h>

#include <AP_Logger/AP_Logger.h>

AP_DAL_Beacon::AP_DAL_Beacon()
{
    for (uint8_t i=0; i<ARRAY_SIZE(_RBCI); i++) {
        _RBCI[i].instance = i;
    }
}

void AP_DAL_Beacon::start_frame(const uint64_t time_us)
{
    const auto *beacon = AP::beacon();

    _RBCH.ptr_is_nullptr = (beacon == nullptr);
    const log_RBCH old = _RBCH;
    if (beacon != nullptr) {
        _RBCH.count = beacon->count();
        _RBCH.get_vehicle_position_ned_returncode = beacon->get_vehicle_position_ned(_RBCH.vehicle_position_ned, _RBCH.accuracy_estimate);

        _RBCH.get_origin_returncode = beacon->get_origin(_origin);
        _RBCH.origin_lat = _origin.lat;
        _RBCH.origin_lng = _origin.lng;
        _RBCH.origin_alt = _origin.alt;
    }
    if (STRUCT_NEQ(old, _RBCH)) {
        WRITE_REPLAY_BLOCK(RBCH, _RBCH);
    }
    if (beacon == nullptr) {
        return;
    }

    for (uint8_t i=0; i<ARRAY_SIZE(_RBCI); i++) {
        log_RBCI &RBCI = _RBCI[i];
        const log_RBCI old_RBCI = RBCI;
        const uint32_t last_update_ms = beacon->beacon_last_update_ms(i);
        if (last_update_ms == _last_logged_update_ms[i]) {
            continue;
        }
        _last_logged_update_ms[i] = last_update_ms;
        RBCI.last_update_ms = last_update_ms;
        RBCI.position = beacon->beacon_position(i);
        RBCI.distance = beacon->beacon_distance(i);
        RBCI.healthy = beacon->beacon_healthy(i);

        if (STRUCT_NEQ(old_RBCI, RBCI)) {
            WRITE_REPLAY_BLOCK(RBCI, RBCI);
        }
    }
}
