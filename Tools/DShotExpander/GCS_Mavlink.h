#pragma once

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_DShotExpander : public GCS_MAVLINK
{

public:

    void data_stream_send(void) override;
    uint8_t sysid_my_gcs(void) const override { return 0; }

protected:
    AP_Mission *get_mission() override { return nullptr; }
    AP_Rally *get_rally() const override { return nullptr; }
    Compass *get_compass() const override { return nullptr; }
    class AP_Camera *get_camera() const override { return nullptr; }
    bool set_mode(uint8_t mode) override { return false; }
    const AP_FWVersion &get_fwver() const;
    
    void set_ekf_origin(const Location& loc) override {}
    MAV_TYPE frame_type() const override { return MAV_TYPE_GENERIC; }
    MAV_MODE base_mode() const override { return (MAV_MODE)0; }
    uint32_t custom_mode() const override { return 0; }
    MAV_STATE system_status() const override { return (MAV_STATE)0; }
    uint32_t telem_delay() const override { return 0; }

    void handleMessage(mavlink_message_t * msg) override;
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override { return false; };
    void handle_change_alt_request(AP_Mission::Mission_Command &cmd) override {};
};
