#include "AP_DAL.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>

extern const AP_HAL::HAL& hal;

enum class FrameItem : uint8_t {
    AVAILABLE_MEMORY = 0,
};

AP_DAL *AP_DAL::_singleton = nullptr;

void AP_DAL::start_frame(AP_DAL::FrameType frametype)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)

    const AP_AHRS &ahrs = AP::ahrs();

    _RFRH.time_us = AP_HAL::micros64();
    _RFRH.time_flying_ms = AP::vehicle()->get_time_flying_ms();
    WRITE_REPLAY_BLOCK(RFRH, _RFRH);

    // update RFRN data
    const log_RFRN old = _RFRN;
    // update RFRH data
    _RFRN.state_bitmask = 0;
    if (hal.util->get_soft_armed()) {
        _RFRN.state_bitmask |= uint8_t(StateMask::ARMED);
    }
    _home = ahrs.get_home();
    _RFRN.lat = _home.lat;
    _RFRN.lng = _home.lng;
    _RFRN.alt = _home.alt;
    _RFRN.get_compass_is_null = AP::ahrs().get_compass() == nullptr;
    _RFRN.rangefinder_ptr_is_null = AP::rangefinder() == nullptr;
    _RFRN.airspeed_ptr_is_null = AP::airspeed() == nullptr;
    _RFRN.EAS2TAS = AP::baro().get_EAS2TAS();
    _RFRN.vehicle_class = ahrs.get_vehicle_class();
    _RFRN.fly_forward = ahrs.get_fly_forward();
    _RFRN.ahrs_airspeed_sensor_enabled = AP::ahrs().airspeed_sensor_enabled();
    _RFRN.available_memory = hal.util->available_memory();
    if (STRUCT_NEQ(old, _RFRN)) {
        WRITE_REPLAY_BLOCK(RFRN, _RFRN);
    }

    // update body conversion
    _rotation_vehicle_body_to_autopilot_body = ahrs.get_rotation_vehicle_body_to_autopilot_body();

    _ins.start_frame(_RFRH.time_us);
    _baro.start_frame(_RFRH.time_us);
    _gps.start_frame(_RFRH.time_us);
    _compass.start_frame(_RFRH.time_us);
    _airspeed.start_frame(_RFRH.time_us);
    _rangefinder.start_frame(_RFRH.time_us);
    _beacon.start_frame(_RFRH.time_us);
    _visualodom.start_frame(_RFRH.time_us);

    const log_RFRF old_RFRF = _RFRF;
    _RFRF.frame_type = (uint8_t)frametype;
    if (STRUCT_NEQ(old_RFRF, _RFRF)) {
        WRITE_REPLAY_BLOCK(RFRF, _RFRF);
    }

    // populate some derivative values:
    _micros = _RFRH.time_us;
    _millis = _RFRH.time_us / 1000UL;

    _trim = ahrs.get_trim();
#endif
}

void AP_DAL::log_event2(AP_DAL::Event2 event)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    const struct log_REV2 pkt{
        event          : uint8_t(event),
    };
    WRITE_REPLAY_BLOCK(REV2, pkt);
#endif
}

void AP_DAL::log_SetOriginLLH2(const Location &loc)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    const struct log_RSO2 pkt{
        lat            : loc.lat,
        lng            : loc.lng,
        alt            : loc.alt,
    };
    WRITE_REPLAY_BLOCK(RSO2, pkt);
#endif
}

void AP_DAL::log_writeDefaultAirSpeed2(const float airspeed)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    const struct log_RWA2 pkt{
        airspeed:      airspeed,
    };
    WRITE_REPLAY_BLOCK(RWA2, pkt);
#endif
}

void AP_DAL::log_event3(AP_DAL::Event3 event)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    const struct log_REV3 pkt{
        event          : uint8_t(event),
    };
    WRITE_REPLAY_BLOCK(REV3, pkt);
#endif
}

void AP_DAL::log_SetOriginLLH3(const Location &loc)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    const struct log_RSO3 pkt{
        lat            : loc.lat,
        lng            : loc.lng,
        alt            : loc.alt,
    };
    WRITE_REPLAY_BLOCK(RSO3, pkt);
#endif
}

void AP_DAL::log_writeDefaultAirSpeed3(const float airspeed)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    const struct log_RWA3 pkt{
        airspeed:      airspeed,
    };
    WRITE_REPLAY_BLOCK(RWA3, pkt);
#endif
}

void AP_DAL::log_writeEulerYawAngle(float yawAngle, float yawAngleErr, uint32_t timeStamp_ms, uint8_t type)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    const struct log_REY3 pkt{
        yawangle       : yawAngle,
        yawangleerr    : yawAngleErr,
        timestamp_ms   : timeStamp_ms,
        type           : type,
    };
    WRITE_REPLAY_BLOCK(REY3, pkt);
#endif
}

int AP_DAL::snprintf(char* str, size_t size, const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int res = hal.util->vsnprintf(str, size, format, ap);
    va_end(ap);
    return res;
}

void *AP_DAL::malloc_type(size_t size, Memory_Type mem_type)
{
    return hal.util->malloc_type(size, AP_HAL::Util::Memory_Type(mem_type));
}


const AP_DAL_Compass *AP_DAL::get_compass() const
{
    if (_RFRN.get_compass_is_null) {
        return nullptr;
    }
    return &_compass;
}

// map core number for replay
uint8_t AP_DAL::logging_core(uint8_t c) const
{
#if APM_BUILD_TYPE(APM_BUILD_Replay)
    return c+100U;
#else
    return c;
#endif
}

#include <stdio.h>

namespace AP {

AP_DAL &dal()
{
    return *AP_DAL::get_singleton();
}

};
