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

    // update RFRH data
    _RFRH.state_bitmask = 0;
    if (hal.util->get_soft_armed()) {
        _RFRH.state_bitmask |= uint8_t(StateMask::ARMED);
    }

    auto &logger = AP::logger();

    const AP_AHRS &ahrs = AP::ahrs();

    _RFRH.frame_number++;
    _RFRH.time_us = AP_HAL::micros64();
    _RFRH.get_compass_is_null = AP::ahrs().get_compass() == nullptr;
    _RFRH.rangefinder_ptr_is_null = AP::rangefinder() == nullptr;
    _RFRH.airspeed_ptr_is_null = AP::airspeed() == nullptr;
    _RFRH.EAS2TAS = AP::baro().get_EAS2TAS();
    _RFRH.vehicle_class = ahrs.get_vehicle_class();
    _RFRH.fly_forward = ahrs.get_fly_forward();
    _trim = ahrs.get_trim();
    _RFRH.trimx = _trim.x;
    _RFRH.trimy = _trim.y;
    _RFRH.trimz = _trim.z;
    _RFRH.ahrs_airspeed_sensor_enabled = AP::ahrs().airspeed_sensor_enabled();
    _RFRH.available_memory = hal.util->available_memory();
    _RFRH.time_flying_ms = AP::vehicle()->get_time_flying_ms();
    logger.WriteReplayBlock((void*)&_RFRH, sizeof(_RFRH));

    // update RFRN data
    _home = ahrs.get_home();
    _RFRN.lat = _home.lat;
    _RFRN.lng = _home.lng;
    _RFRN.alt = _home.alt;
    logger.WriteReplayBlock((void*)&_RFRN, sizeof(_RFRN));

    // update RFRR data:
    _rotation_vehicle_body_to_autopilot_body = ahrs.get_rotation_vehicle_body_to_autopilot_body();
    _RFRR.m0 = _rotation_vehicle_body_to_autopilot_body[0][0];
    _RFRR.m1 = _rotation_vehicle_body_to_autopilot_body[0][1];
    _RFRR.m2 = _rotation_vehicle_body_to_autopilot_body[0][2];
    _RFRR.m3 = _rotation_vehicle_body_to_autopilot_body[1][0];
    _RFRR.m4 = _rotation_vehicle_body_to_autopilot_body[1][1];
    _RFRR.m5 = _rotation_vehicle_body_to_autopilot_body[1][2];
    _RFRR.m6 = _rotation_vehicle_body_to_autopilot_body[2][0];
    _RFRR.m7 = _rotation_vehicle_body_to_autopilot_body[2][1];
    _RFRR.m8 = _rotation_vehicle_body_to_autopilot_body[2][2];
    // consider logging the rotation, which shouldn't change too much...
    if (memcmp((void*)&_last_logged_RFRR, (void*)&_RFRR, sizeof(_RFRR)) != 0) {
        logger.WriteReplayBlock((void*)&_RFRR, sizeof(_RFRR));
        _last_logged_RFRR = _RFRR;
    }

    _ins.start_frame(_RFRH.time_us);
    _baro.start_frame(_RFRH.time_us);
    _gps.start_frame(_RFRH.time_us);
    _compass.start_frame(_RFRH.time_us);
    _airspeed.start_frame(_RFRH.time_us);
    _rangefinder.start_frame(_RFRH.time_us);

    _RFRF.time_us = _RFRH.time_us;
    _RFRF.frame_type = (uint8_t)frametype;
    logger.WriteReplayBlock((void*)&_RFRF, sizeof(_RFRF));

    // populate some derivative values:
    _micros = _RFRH.time_us;
    _millis = _RFRH.time_us / 1000UL;

#endif
}

void AP_DAL::log_event2(AP_DAL::Event2 event)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    const struct log_REV2 pkt{
        LOG_PACKET_HEADER_INIT(LOG_REV2_MSG),
        time_us        : _RFRH.time_us,
        event          : uint8_t(event),
    };
    AP::logger().WriteReplayBlock((void*)&pkt, sizeof(pkt));
#endif
}

void AP_DAL::log_SetOriginLLH2(const Location &loc)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    const struct log_RSO2 pkt{
        LOG_PACKET_HEADER_INIT(LOG_RSO2_MSG),
        time_us        : _RFRH.time_us,   // this isn't correct?
        lat            : loc.lat,
        lng            : loc.lng,
        alt            : loc.alt,
    };
    AP::logger().WriteReplayBlock((void*)&pkt, sizeof(pkt));
#endif
}

void AP_DAL::log_writeDefaultAirSpeed2(const float airspeed)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    const struct log_RWA2 pkt{
        LOG_PACKET_HEADER_INIT(LOG_RWA2_MSG),
        time_us        : _RFRH.time_us,   // this isn't correct?
        airspeed:      airspeed,
    };
    AP::logger().WriteReplayBlock((void*)&pkt, sizeof(pkt));
#endif
}

void AP_DAL::log_event3(AP_DAL::Event3 event)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    const struct log_REV3 pkt{
        LOG_PACKET_HEADER_INIT(LOG_REV3_MSG),
        time_us        : _RFRH.time_us,
        event          : uint8_t(event),
    };
    AP::logger().WriteReplayBlock((void*)&pkt, sizeof(pkt));
#endif
}

void AP_DAL::log_SetOriginLLH3(const Location &loc)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    const struct log_RSO3 pkt{
        LOG_PACKET_HEADER_INIT(LOG_RSO3_MSG),
        time_us        : _RFRH.time_us,   // this isn't correct?
        lat            : loc.lat,
        lng            : loc.lng,
        alt            : loc.alt,
    };
    AP::logger().WriteReplayBlock((void*)&pkt, sizeof(pkt));
#endif
}

void AP_DAL::log_writeDefaultAirSpeed3(const float airspeed)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    const struct log_RWA3 pkt{
        LOG_PACKET_HEADER_INIT(LOG_RWA3_MSG),
        time_us        : _RFRH.time_us,   // this isn't correct?
        airspeed:      airspeed,
    };
    AP::logger().WriteReplayBlock((void*)&pkt, sizeof(pkt));
#endif
}

void AP_DAL::log_writeEulerYawAngle(float yawAngle, float yawAngleErr, uint32_t timeStamp_ms, uint8_t type)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    const struct log_REY3 pkt{
        LOG_PACKET_HEADER_INIT(LOG_REY3_MSG),
        time_us        : _RFRH.time_us,   // this isn't correct?
        yawangle       : yawAngle,
        yawangleerr    : yawAngleErr,
        timestamp_ms   : timeStamp_ms,
        type           : type,
    };
    AP::logger().WriteReplayBlock((void*)&pkt, sizeof(pkt));
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
    if (_RFRH.get_compass_is_null) {
        return nullptr;
    }
    return &_compass;
}

#include <stdio.h>

namespace AP {

AP_DAL &dal()
{
    return *AP_DAL::get_singleton();
}

};
