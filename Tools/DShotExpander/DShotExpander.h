#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Scheduler/AP_Scheduler.h>
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
    
    // GCS selection
    GCS_DShotExpander _gcs;
    GCS_DShotExpander &gcs() { return _gcs; }
};

extern const AP_HAL::HAL& hal;
extern DShotExpander dshotexpander;
