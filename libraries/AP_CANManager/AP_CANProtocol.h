#pragma once
#include <AP_HAL/AP_HAL.h>

namespace AP {
class CANProtocol {
public:
    /* method called when initializing the CAN interfaces
     *
     * if initialization doesn't have errors, protocol class
     * should create a thread to do send and receive operations
     */
    virtual void init(uint8_t driver_index, bool enable_filters) = 0;
    virtual bool add_interface(AP_HAL::CANDriver* can_iface) = 0;
};
}