
#pragma once

#include "AP_Networking_Backend.h"

#ifndef AP_NETWORKING_PING_ENABLED
#define AP_NETWORKING_PING_ENABLED AP_NETWORKING_ENABLED
#endif

#if AP_NETWORKING_PING_ENABLED

class AP_Networking_Ping : public AP_Networking_Backend {
public:
    AP_Networking_Ping(AP_Networking &front,
                        AP_Networking::AP_Networking_State &state,
                        AP_Networking_Params &params);

    void init() override;

    void update() override;

    static const struct AP_Param::GroupInfo var_info[];

private:
    
    // void send_data();
    // bool report_stats();

    struct {
        AP_Int16 ip[4];
        AP_Int16 duration_seconds;
    } _params;

    struct udp_pcb *_pcb;

    struct {
        uint32_t start_ms;
        uint32_t gcs_send_ms;
        
        // uint64_t packets_total;
        // uint64_t packets_per_sec;
        // uint64_t packets_per_sec_max;

        // uint64_t bytes_total;
        // uint64_t bytes_per_sec;
        // uint64_t bytes_per_sec_max;

    } _stats;

};

#endif // AP_NETWORKING_PING_ENABLED
