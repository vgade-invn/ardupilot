#include "EventHandle.h"
#include <AP_HAL/AP_HAL.h>

int32_t ChibiOS::EventHandle::wait(uint64_t duration)
{
    msg_t ret = msg_t();

    if (duration == 0) {
        ret = sem_.wait(TIME_IMMEDIATE);
    }
    else {
        ret = sem_.wait(chTimeUS2I(duration));
    }
    return ret == MSG_OK;
}

void ChibiOS::EventHandle::signal()
{
    sem_.signal();
}

void ChibiOS::EventHandle::signalFromInterrupt()
{
    chSysLockFromISR();
    sem_.signalI();
    chSysUnlockFromISR();
}
