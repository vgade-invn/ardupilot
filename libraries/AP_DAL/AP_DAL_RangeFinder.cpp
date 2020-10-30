#include "AP_DAL_RangeFinder.h"

#include <AP_Logger/AP_Logger.h>

#include <AP_RangeFinder/AP_RangeFinder_Backend.h>

AP_DAL_RangeFinder::AP_DAL_RangeFinder()
{
    _RRNH.head1 = HEAD_BYTE1;
    _RRNH.head2 = HEAD_BYTE2;
    _RRNH.msgid = LOG_RRNH_MSG;

    for (uint8_t i=0; i<ARRAY_SIZE(_RRNI); i++) {
        _RRNI[i].head1 = HEAD_BYTE1;
        _RRNI[i].head2 = HEAD_BYTE2;
        _RRNI[i].msgid = LOG_RRNI_MSG;
        _RRNI[i].instance = i;
    }

    for (uint8_t i=0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        // this avoids having to discard a const....
        _backend[i] = new AP_DAL_RangeFinder_Backend(_RRNI[i]);
    }
}

int16_t AP_DAL_RangeFinder::ground_clearance_cm_orient(enum Rotation orientation) const
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone)
    const auto *rangefinder = AP::rangefinder();

    if (orientation != ROTATION_PITCH_270) {
        // the EKF only asks for this from a specific orientation.  Thankfully.
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return rangefinder->ground_clearance_cm_orient(orientation);
    }
#endif

    return _RRNH.ground_clearance_cm;
}

int16_t AP_DAL_RangeFinder::max_distance_cm_orient(enum Rotation orientation) const
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone)
    const auto *rangefinder = AP::rangefinder();

    if (orientation != ROTATION_PITCH_270) {
        // the EKF only asks for this from a specific orientation.  Thankfully.
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return rangefinder->ground_clearance_cm_orient(orientation);
    }
#endif

    return _RRNH.max_distance_cm;
}

void AP_DAL_RangeFinder::start_frame(const uint64_t time_us)
{
    const auto *rangefinder = AP::rangefinder();
    if (rangefinder == nullptr) {
        return;
    }

    auto &logger = AP::logger();

    // EKF only asks for this *down*.
    _RRNH.ground_clearance_cm = rangefinder->ground_clearance_cm_orient(ROTATION_PITCH_270);
    _RRNH.max_distance_cm = rangefinder->ground_clearance_cm_orient(ROTATION_PITCH_270);
    logger.WriteReplayBlock((void*)&_RRNH, sizeof(_RRNH));

    for (uint8_t i=0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        auto *backend = rangefinder->get_backend(i);
        _get_backend_returnptr[i] = backend;
        if (backend == nullptr) {
            return;
        }
        _backend[i]->start_frame(time_us, backend);
    }
}




AP_DAL_RangeFinder_Backend::AP_DAL_RangeFinder_Backend(struct log_RRNI &RRNI) :
    _RRNI(RRNI)
{
}

void AP_DAL_RangeFinder_Backend::start_frame(uint64_t time_us, AP_RangeFinder_Backend *backend) {
    auto &logger = AP::logger();

    _RRNI.time_us = time_us;
    _RRNI.orientation = backend->orientation();
    _RRNI.status = (uint8_t)backend->status();
    _RRNI.pos_offset.from_Vector3f(backend->get_pos_offset());
    logger.WriteReplayBlock((void*)&_RRNI, sizeof(_RRNI));
}


AP_DAL_RangeFinder_Backend *AP_DAL_RangeFinder::get_backend(uint8_t id) const
{
   if (id > RANGEFINDER_MAX_INSTANCES) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return nullptr;
    }

   if (_get_backend_returnptr[id] == nullptr) {
       return nullptr;
   }

   return _backend[id];
}
