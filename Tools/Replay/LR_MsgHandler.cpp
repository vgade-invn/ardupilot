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
    AP::dal().handle_message(*((log_RFRH*)(msg+3)));
}

void LR_MsgHandler_RFRF::process_message(uint8_t *msg)
{
    const log_RFRF &rfrf = *((log_RFRF*)(msg+3));
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
    const log_REV2 &rev2 = *((log_REV2*)(msg+3));

    switch ((AP_DAL::Event2)rev2.event) {

    case AP_DAL::Event2::resetGyroBias:
        ekf2.resetGyroBias();
        break;
    case AP_DAL::Event2::resetHeightDatum:
        ekf2.resetHeightDatum();
        break;
    case AP_DAL::Event2::setInhibitGPS:
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
    case AP_DAL::Event2::LoggingDone:
        ekf2.Log_Write();
        break;
    }
}

void LR_MsgHandler_RSO2::process_message(uint8_t *msg)
{
    const log_RSO2 &rso2 = *((log_RSO2*)(msg+3));
    Location loc;
    loc.lat = rso2.lat;
    loc.lng = rso2.lng;
    loc.alt = rso2.alt;
    ekf2.setOriginLLH(loc);
}

void LR_MsgHandler_RWA2::process_message(uint8_t *msg)
{
    const log_RWA2 &rwa2 = *((log_RWA2*)(msg+3));
    ekf2.writeDefaultAirSpeed(rwa2.airspeed);
}


void LR_MsgHandler_REV3::process_message(uint8_t *msg)
{
    const log_REV3 &rev3 = *((log_REV3*)(msg+3));

    switch ((AP_DAL::Event3)rev3.event) {

    case AP_DAL::Event3::resetGyroBias:
        ekf3.resetGyroBias();
        break;
    case AP_DAL::Event3::resetHeightDatum:
        ekf3.resetHeightDatum();
        break;
    case AP_DAL::Event3::setInhibitGPS:
        ekf3.setInhibitGPS();
        break;
    case AP_DAL::Event3::setTakeoffExpected:
        ekf3.setTakeoffExpected(true);
        break;
    case AP_DAL::Event3::unsetTakeoffExpected:
        ekf3.setTakeoffExpected(false);
        break;
    case AP_DAL::Event3::setTouchdownExpected:
        ekf3.setTouchdownExpected(true);
        break;
    case AP_DAL::Event3::unsetTouchdownExpected:
        ekf3.setTouchdownExpected(false);
        break;
    case AP_DAL::Event3::setInhibitGpsVertVelUse:
        ekf3.setInhibitGpsVertVelUse(true);
        break;
    case AP_DAL::Event3::unsetInhibitGpsVertVelUse:
        ekf3.setInhibitGpsVertVelUse(false);
        break;
    case AP_DAL::Event3::setTerrainHgtStable:
        ekf3.setTerrainHgtStable(true);
        break;
    case AP_DAL::Event3::unsetTerrainHgtStable:
        ekf3.setTerrainHgtStable(false);
        break;
    case AP_DAL::Event3::requestYawReset:
        ekf3.requestYawReset();
        break;
    case AP_DAL::Event3::LoggingDone:
        ekf3.Log_Write();
        break;
    case AP_DAL::Event3::checkLaneSwitch:
        ekf3.checkLaneSwitch();
        break;
    case AP_DAL::Event3::updateCoreErrorScores:
        ekf3.updateCoreErrorScores();
        break;
    case AP_DAL::Event3::updateCoreRelativeErrors:
        ekf3.updateCoreRelativeErrors();
        break;
    case AP_DAL::Event3::resetCoreErrors:
        ekf3.resetCoreErrors();
        break;
    }
}

void LR_MsgHandler_RSO3::process_message(uint8_t *msg)
{
    const log_RSO3 &rso3 = *((log_RSO3*)(msg+3));
    Location loc;
    loc.lat = rso3.lat;
    loc.lng = rso3.lng;
    loc.alt = rso3.alt;
    ekf3.setOriginLLH(loc);
}

void LR_MsgHandler_RWA3::process_message(uint8_t *msg)
{
    const log_RWA3 &rwa3 = *((log_RWA3*)(msg+3));
    ekf3.writeDefaultAirSpeed(rwa3.airspeed);
}

void LR_MsgHandler_REY3::process_message(uint8_t *msg)
{
    const log_RWA3 &rwa3 = *((log_RWA3*)(msg+3));
    ekf3.writeDefaultAirSpeed(rwa3.airspeed);
}

void LR_MsgHandler_RFRR::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RFRR*)(msg+3)));
}

void LR_MsgHandler_RISH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RISH*)(msg+3)));
}
void LR_MsgHandler_RISI::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RISI*)(msg+3)));
}
void LR_MsgHandler_RISJ::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RISJ*)(msg+3)));
}

void LR_MsgHandler_RASH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RASH*)(msg+3)));
}
void LR_MsgHandler_RASI::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RASI*)(msg+3)));
}

void LR_MsgHandler_RBRH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RBRH*)(msg+3)));
}
void LR_MsgHandler_RBRI::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RBRI*)(msg+3)));
}

void LR_MsgHandler_RRNH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RRNH*)(msg+3)));
}
void LR_MsgHandler_RRNI::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RRNI*)(msg+3)));
}

void LR_MsgHandler_RGPH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RGPH*)(msg+3)));
}
void LR_MsgHandler_RGPI::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RGPI*)(msg+3)));
}
void LR_MsgHandler_RGPJ::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RGPJ*)(msg+3)));
}

void LR_MsgHandler_RMGH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RMGH*)(msg+3)));
}
void LR_MsgHandler_RMGI::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RMGI*)(msg+3)));
}

void LR_MsgHandler_RBCH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RBCH*)(msg+3)));
}
void LR_MsgHandler_RBCI::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RBCI*)(msg+3)));
}

void LR_MsgHandler_RVOH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(*((log_RVOH*)(msg)));
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
