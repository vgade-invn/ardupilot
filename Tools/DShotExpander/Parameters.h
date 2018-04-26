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
    };

    AP_Int16 format_version;

    Parameters() {}
};

extern const AP_Param::Info var_info[];
