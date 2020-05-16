#pragma once

#include <AP_HAL/AP_HAL.h>
#include <uavcan/uavcan.hpp>

namespace uavcan {

class CanDriver;

class CanIface : public ICanIface, Noncopyable
{
    friend class CanDriver;
    AP_HAL::CANDriver* can_drv_;
public:
    CanIface(AP_HAL::CANDriver *can_drv) : can_drv_(can_drv) {}

    virtual int16_t send(const CanFrame& frame, MonotonicTime tx_deadline, 
                            CanIOFlags flags) override;
    
    virtual int16_t receive(CanFrame& out_frame, MonotonicTime& out_ts_monotonic, 
                            UtcTime& out_ts_utc, CanIOFlags& out_flags) override;

    int16_t configureFilters(const CanFilterConfig* filter_configs, 
                                uint16_t num_configs) override;

    uint16_t getNumFilters() const override;

    uint64_t getErrorCount() const override;
};

/**
 * Generic CAN driver.
 */
class CanDriver : public ICanDriver, Noncopyable
{
    CanIface* ifaces[MAX_NUMBER_OF_CAN_INTERFACES];
    uint8_t num_ifaces;
    HAL_EventHandle _event_handle;
public:
    bool add_interface(AP_HAL::CANDriver *can_drv);

    ICanIface* getIface(uint8_t iface_index) override;

    uint8_t getNumIfaces() const override;

    CanSelectMasks makeSelectMasks(const CanFrame* (& pending_tx)[MaxCanIfaces]) const;
    
    int16_t select(CanSelectMasks& inout_masks,
                                    const CanFrame* (& pending_tx)[MaxCanIfaces],
                                    const MonotonicTime blocking_deadline) override;
};

}
