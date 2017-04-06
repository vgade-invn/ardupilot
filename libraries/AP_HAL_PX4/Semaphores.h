#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "AP_HAL_PX4.h"

class PX4::Semaphore : public AP_HAL::Semaphore {
public:
    Semaphore();
    bool give();
    bool take(uint32_t timeout_ms);
    bool take_nonblocking();
private:
    sem_t sem;
};
#endif // CONFIG_HAL_BOARD
