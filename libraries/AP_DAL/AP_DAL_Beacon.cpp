#include "AP_DAL_Beacon.h"

#include <AP_Beacon/AP_Beacon.h>

#include <AP_Logger/AP_Logger.h>

AP_DAL_Beacon::AP_DAL_Beacon()
{
    _RBCH.head1 = HEAD_BYTE1;
    _RBCH.head2 = HEAD_BYTE2;
    _RBCH.msgid = LOG_RBCH_MSG;

    for (uint8_t i=0; i<ARRAY_SIZE(_RBCI); i++) {
        _RBCI[i].head1 = HEAD_BYTE1;
        _RBCI[i].head2 = HEAD_BYTE2;
        _RBCI[i].msgid = LOG_RBCI_MSG;
        _RBCI[i].instance = i;
    }
}

void AP_DAL_Beacon::start_frame(const uint64_t time_us)
{
    const auto *beacon = AP::beacon();

    auto &logger = AP::logger();

    _RBCH.time_us = time_us;
    _RBCH.ptr_is_nullptr = (beacon == nullptr);
    if (beacon != nullptr) {
        _RBCH.count = beacon->count();
        Vector3f tmp;
        float tmp_aa;
        _RBCH.get_vehicle_position_ned_returncode = beacon->get_vehicle_position_ned(tmp, tmp_aa);
        _RBCH.vehicle_position_ned.from_Vector3f(tmp);
        _RBCH.accuracy_estimate = tmp_aa;

        _RBCH.get_origin_returncode = beacon->get_origin(_origin);
        _RBCH.origin_lat = _origin.lat;
        _RBCH.origin_lng = _origin.lng;
        _RBCH.origin_alt = _origin.alt;
    }
    logger.WriteReplayBlock((void*)&_RBCH, sizeof(_RBCH));
    if (beacon == nullptr) {
        return;
    }

    for (uint8_t i=0; i<ARRAY_SIZE(_RBCI); i++) {
        log_RBCI &RBCI = _RBCI[i];
        const uint32_t last_update_ms = beacon->beacon_last_update_ms(i);
        if (last_update_ms == _last_logged_update_ms[i]) {
            continue;
        }
        _last_logged_update_ms[i] = last_update_ms;
        RBCI.time_us = time_us;
        RBCI.last_update_ms = last_update_ms;
        RBCI.position.from_Vector3f(beacon->beacon_position(i));
        RBCI.distance = beacon->beacon_distance(i);
        RBCI.healthy = beacon->beacon_healthy(i);

        logger.WriteReplayBlock((void*)&RBCI, sizeof(RBCI));
    }
}
