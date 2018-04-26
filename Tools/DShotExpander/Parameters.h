#pragma once

#include <AP_Common/AP_Common.h>

// Global parameter class.
//
class Parameters {
public:

    static const uint16_t k_format_version = 1;

    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,
        k_param_scheduler,
        k_param_serial_manager,
        k_param_DataFlash,
        k_param_log_bitmask,
        k_param_servo_channels,
        k_param_BoardConfig,
    };

    AP_Int16 format_version;
    AP_Int32 log_bitmask;

    Parameters() {}
};

extern const AP_Param::Info var_info[];
