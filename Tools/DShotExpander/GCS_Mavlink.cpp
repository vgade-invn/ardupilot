#include "DShotExpander.h"

#include "GCS_Mavlink.h"

// try to send a message, return false if it won't fit in the serial tx buffer
bool GCS_MAVLINK_DShotExpander::try_send_message(enum ap_message id)
{
    switch (id) {
    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        send_heartbeat();
        return true;
    default:
        return GCS_MAVLINK::try_send_message(id);
    }
    return false;
}

void DShotExpander::gcs_send_heartbeat(void)
{
    gcs().send_message(MSG_HEARTBEAT);
}

void DShotExpander::gcs_send_deferred(void)
{
    gcs().retry_deferred();
}

void DShotExpander::gcs_data_stream_send(void)
{
    gcs().data_stream_send();
}

const AP_Param::GroupInfo GCS_MAVLINK::var_info[] = {
    // @Param: RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and SENSOR_OFFSETS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK, streamRates[0],  0),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Stream rate of SYS_STATUS, POWER_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, and FENCE_STATUS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK, streamRates[1],  0),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate to ground station
    // @Description: Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK, streamRates[2],  0),

    // @Param: RAW_CTRL
    // @DisplayName: Raw Control stream rate to ground station
    // @Description: Stream rate of RC_CHANNELS_SCALED (HIL only) to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK, streamRates[3],  0),

    // @Param: POSITION
    // @DisplayName: Position stream rate to ground station
    // @Description: Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK, streamRates[4],  0),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: Stream rate of ATTITUDE, SIMSTATE (SITL only), AHRS2 and PID_TUNING to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK, streamRates[5],  0),

    // @Param: EXTRA2
    // @DisplayName: Extra data type 2 stream rate to ground station
    // @Description: Stream rate of VFR_HUD to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK, streamRates[6],  0),

    // @Param: EXTRA3
    // @DisplayName: Extra data type 3 stream rate to ground station
    // @Description: Stream rate of AHRS, HWSTATUS, SYSTEM_TIME, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, BATTERY2, MOUNT_STATUS, OPTICAL_FLOW, GIMBAL_REPORT, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION and RPM to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK, streamRates[7],  0),

    // @Param: PARAMS
    // @DisplayName: Parameter stream rate to ground station
    // @Description: Stream rate of PARAM_VALUE to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK, streamRates[8],  0),

    // @Param: ADSB
    // @DisplayName: ADSB stream rate to ground station
    // @Description: ADSB stream rate to ground station
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ADSB",   9, GCS_MAVLINK, streamRates[9],  5),
    AP_GROUPEND
};

void
GCS_MAVLINK_DShotExpander::data_stream_send(void)
{
    gcs().set_out_of_time(false);

    send_queued_parameters();
}


void GCS_MAVLINK_DShotExpander::handleMessage(mavlink_message_t* msg)
{
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: {
        handle_request_data_stream(msg, false);
        break;
    }

    case MAVLINK_MSG_ID_COMMAND_LONG: {
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(msg, &packet);
        
        MAV_RESULT result = MAV_RESULT_UNSUPPORTED;
        
        switch (packet.command) {
        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            if (is_equal(packet.param1, 1.0f) || is_equal(packet.param1, 3.0f)) {
                // when packet.param1 == 3 we reboot to hold in bootloader
                hal.scheduler->reboot(is_equal(packet.param1, 3.0f));
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        default:
            result = handle_command_long_message(packet);
            break;
        }

        mavlink_msg_command_ack_send_buf(
            msg,
            chan,
            packet.command,
            result);
        break;
    }
        
    default:
        handle_common_message(msg);
        break;
    }
}

/*
 *  look for incoming commands on the GCS links
 */
void DShotExpander::gcs_check_input(void)
{
    gcs().update();
}

const AP_FWVersion &GCS_MAVLINK_DShotExpander::get_fwver() const
{
    return dshotexpander.fwver;
}

