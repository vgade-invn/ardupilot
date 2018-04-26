#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "Parameters.h"
#include "GCS_DShotExpander.h"


class DShotExpander : public AP_HAL::HAL::Callbacks {
public:
    friend class GCS_MAVLINK_DShotExpander;
    DShotExpander(void);

    // HAL::Callbacks implementation.
    void setup() override;
    void loop() override;

private:
    AP_Param param_loader;

    static const AP_Param::Info var_info[];
    Parameters g;

    // main loop scheduler
    AP_Scheduler scheduler;
    static const AP_Scheduler::Task scheduler_tasks[];
    
    void load_parameters();
    void fast_loop();
    void one_hz_loop();
    void gcs_send_heartbeat();
    void gcs_send_deferred(void);
    void gcs_data_stream_send(void);
    void gcs_check_input();

    static const AP_FWVersion fwver;
    AP_SerialManager serial_manager;

    // Dataflash
    DataFlash_Class DataFlash;

    SRV_Channels servo_channels;
    AP_BoardConfig BoardConfig;
    
    // GCS selection
    GCS_DShotExpander _gcs;
    GCS_DShotExpander &gcs() { return _gcs; }

    void setup_uart();
    static void uart_start(void *ctx);
    void uart_thread();
    void stop_motors();
    AP_HAL::UARTDriver *uart;

    static const uint8_t max_channels = 8;
    
    uint32_t crc_errors;
    uint32_t pkt_errors;
    uint32_t last_packet_ms;
};

extern const AP_HAL::HAL& hal;
extern DShotExpander dshotexpander;
