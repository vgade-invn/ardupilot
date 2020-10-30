#include "AP_DAL_Baro.h"

#include <AP_Logger/AP_Logger.h>

AP_DAL_Baro::AP_DAL_Baro()
{
    for (uint8_t i=0; i<BARO_MAX_INSTANCES; i++) {
        _RBRI[i].instance = i;
    }
}

void AP_DAL_Baro::start_frame(const uint64_t time_us)
{
    const auto &baro = AP::baro();

    _RBRH.time_us = time_us;
    _RBRH.primary = baro.get_primary();
    _RBRH.num_instances = baro.num_instances();
    WRITE_REPLAY_BLOCK(RBRH, _RBRH);

    for (uint8_t i=0; i<BARO_MAX_INSTANCES; i++) {
        log_RBRI &RBRI = _RBRI[i];
        const uint32_t last_update_ms = baro.get_last_update(i);
        if (last_update_ms == _last_logged_update_ms[i]) {
            continue;
        }
        _last_logged_update_ms[i] = last_update_ms;
        RBRI.time_us = time_us;
        RBRI.last_update_ms = last_update_ms;
        RBRI.healthy = baro.healthy(i);
        RBRI.altitude = baro.get_altitude(i);
        WRITE_REPLAY_BLOCK(RBRI, _RBRI[i]);
    }
}

void AP_DAL_Baro::update_calibration()
{
    AP::baro().update_calibration();
}
