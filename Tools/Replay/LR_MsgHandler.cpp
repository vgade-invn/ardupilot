#include "LR_MsgHandler.h"
#include "LogReader.h"
#include "Replay.h"

#include <AP_HAL_Linux/Scheduler.h>
#include <AP_DAL/AP_DAL.h>

#include <cinttypes>

extern const AP_HAL::HAL& hal;

LR_MsgHandler::LR_MsgHandler(struct log_Format &_f) :
    MsgHandler(_f) {
}

void LR_MsgHandler_RFRH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RFRH*)(msg)));
}

void LR_MsgHandler_RFRF::process_message(uint8_t *msg)
{
    const log_RFRF &rfrf = *((log_RFRF*)(msg));
    AP::dal().handle_message(rfrf);

    switch ((AP_DAL::FrameType)rfrf.frame_type) {

    case AP_DAL::FrameType::InitialiseFilterEKF2:
        ekf2.InitialiseFilter();
        break;
    case AP_DAL::FrameType::UpdateFilterEKF2:
        ekf2.UpdateFilter();
        break;

    case AP_DAL::FrameType::InitialiseFilterEKF3:
        ekf3.InitialiseFilter();
        break;
    case AP_DAL::FrameType::UpdateFilterEKF3:
        ekf3.UpdateFilter();
        break;
    }
}

void LR_MsgHandler_REV2::process_message(uint8_t *msg)
{
    const log_REV2 &rev2 = *((log_REV2*)(msg));

    switch ((AP_DAL::Event2)rev2.event) {

    case AP_DAL::Event2::ResetGyroBias:
        ekf2.resetGyroBias();
        break;
    case AP_DAL::Event2::ResetHeightDatum:
        ekf2.resetHeightDatum();
        break;
    case AP_DAL::Event2::InhibitGPS:
        ekf2.setInhibitGPS();
        break;
    case AP_DAL::Event2::setTakeoffExpected:
        ekf2.setTakeoffExpected(true);
        break;
    case AP_DAL::Event2::unsetTakeoffExpected:
        ekf2.setTakeoffExpected(false);
        break;
    case AP_DAL::Event2::setTouchdownExpected:
        ekf2.setTouchdownExpected(true);
        break;
    case AP_DAL::Event2::unsetTouchdownExpected:
        ekf2.setTouchdownExpected(false);
        break;
    case AP_DAL::Event2::setInhibitGpsVertVelUse:
        ekf2.setInhibitGpsVertVelUse(true);
        break;
    case AP_DAL::Event2::unsetInhibitGpsVertVelUse:
        ekf2.setInhibitGpsVertVelUse(false);
        break;
    case AP_DAL::Event2::setTerrainHgtStable:
        ekf2.setTerrainHgtStable(true);
        break;
    case AP_DAL::Event2::unsetTerrainHgtStable:
        ekf2.setTerrainHgtStable(false);
        break;
    case AP_DAL::Event2::requestYawReset:
        ekf2.requestYawReset();
        break;
    }
}

void LR_MsgHandler_NKF1::process_message(uint8_t *msg)
{
    ekf2.Log_Write();
}

void LR_MsgHandler_XKF1::process_message(uint8_t *msg)
{
    ekf3.Log_Write();
}

void LR_MsgHandler_RFRR::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RFRR*)(msg)));
}

void LR_MsgHandler_RISH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RISH*)(msg)));
}
void LR_MsgHandler_RISI::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RISI*)(msg)));
}
void LR_MsgHandler_RISJ::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RISJ*)(msg)));
}

void LR_MsgHandler_RASH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RASH*)(msg)));
}
void LR_MsgHandler_RASI::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RASI*)(msg)));
}

void LR_MsgHandler_RBRH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RBRH*)(msg)));
}
void LR_MsgHandler_RBRI::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RBRI*)(msg)));
}

void LR_MsgHandler_RRNH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RRNH*)(msg)));
}
void LR_MsgHandler_RRNI::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RRNI*)(msg)));
}

void LR_MsgHandler_RGPH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RGPH*)(msg)));
}
void LR_MsgHandler_RGPI::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RGPI*)(msg)));
}

void LR_MsgHandler_RMGH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RMGH*)(msg)));
}
void LR_MsgHandler_RMGI::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RMGI*)(msg)));
}

#include <AP_AHRS/AP_AHRS.h>
#include "VehicleType.h"

bool LR_MsgHandler_PARM::set_parameter(const char *name, const float value)
{
    const char *ignore_parms[] = {
        // "GPS_TYPE",
        // "AHRS_EKF_TYPE",
        // "EK2_ENABLE",
        // "EK3_ENABLE",
        // "COMPASS_ORIENT",
        // "COMPASS_ORIENT2",
        // "COMPASS_ORIENT3",
        "LOG_FILE_BUFSIZE",
        "LOG_DISARMED"
    };
    for (uint8_t i=0; i < ARRAY_SIZE(ignore_parms); i++) {
        if (strncmp(name, ignore_parms[i], AP_MAX_NAME_SIZE) == 0) {
            ::printf("Ignoring set of %s to %f\n", name, value);
            return true;
        }
    }

    return _set_parameter_callback(name, value);
}

void LR_MsgHandler_PARM::process_message(uint8_t *msg)
{
    const uint8_t parameter_name_len = AP_MAX_NAME_SIZE + 1; // null-term
    char parameter_name[parameter_name_len];
    uint64_t time_us;

    if (field_value(msg, "TimeUS", time_us)) {
    } else {
        // older logs can have a lot of FMT and PARM messages up the
        // front which don't have timestamps.  Since in Replay we run
        // AP_Logger's IO only when stop_clock is called, we can
        // overflow AP_Logger's ringbuffer.  This should force us to
        // do IO:
        hal.scheduler->stop_clock(((Linux::Scheduler*)hal.scheduler)->stopped_clock_usec());
    }

    require_field(msg, "Name", parameter_name, parameter_name_len);

    float value = require_field_float(msg, "Value");
    // if (globals.no_params || replay.check_user_param(parameter_name)) {
    //     printf("Not changing %s to %f\n", parameter_name, value);
    // } else {
    set_parameter(parameter_name, value);
    // }
}
