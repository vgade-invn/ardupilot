// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class OpticalFlow;
class AP_AHRS_NavEKF;

class AP_RangeFinder_OpticalFlow : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_OpticalFlow(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state);

    // static detection function
    static bool detect(RangeFinder &ranger, uint8_t instance);

    // update state
    void update(void);

private:
    uint32_t last_flow_data_ms;
    OpticalFlow *flow;
    AP_AHRS_NavEKF *ahrs;
};
