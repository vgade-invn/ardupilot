#pragma once

#include <stdint.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/EventHandle.h>
#include "AP_HAL_ChibiOS_Namespace.h"
#include <ch.hpp>

class ChibiOS::EventHandle : public AP_HAL::EventHandle {
    chibios_rt::CounterSemaphore sem_;

public:
    EventHandle() : sem_(0)
    {}

    int32_t wait(uint64_t duration) override;

    void signal();

    void signalFromInterrupt();
};