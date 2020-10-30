#include "AP_DAL_Airspeed.h"

#include <AP_Logger/AP_Logger.h>

AP_DAL_Airspeed::AP_DAL_Airspeed()
{
    _RASH.head1 = HEAD_BYTE1;
    _RASH.head2 = HEAD_BYTE2;
    _RASH.msgid = LOG_RASH_MSG;

    for (uint8_t i=0; i<ARRAY_SIZE(_RASI); i++) {
        _RASI[i].head1 = HEAD_BYTE1;
        _RASI[i].head2 = HEAD_BYTE2;
        _RASI[i].msgid = LOG_RASI_MSG;
        _RASI[i].instance = i;
    }
}

void AP_DAL_Airspeed::start_frame(const uint64_t time_us)
{
    const auto *airspeed = AP::airspeed();
    if (airspeed == nullptr) {
        return;
    }

    auto &logger = AP::logger();

    _RASH.time_us = time_us;
    _RASH.num_sensors = airspeed->get_num_sensors();
    _RASH.primary = airspeed->get_primary();
    logger.WriteReplayBlock(&_RASH, sizeof(_RASH));

    for (uint8_t i=0; i<ARRAY_SIZE(_RASI); i++) {
        log_RASI &RASI = _RASI[i];
        const uint32_t last_update_ms = airspeed->last_update_ms(i);
        if (last_update_ms == _last_logged_update_ms[i]) {
            continue;
        }
        _last_logged_update_ms[i] = last_update_ms;
        RASI.time_us = time_us;
        RASI.last_update_ms = last_update_ms;
        RASI.healthy = airspeed->healthy(i);
        RASI.use = airspeed->use(i);
        RASI.airspeed = airspeed->get_airspeed(i);
        logger.WriteReplayBlock(&RASI, sizeof(RASI));
    }
}
