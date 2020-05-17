
#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_HAL/utility/RingBuffer.h"
#include <AP_Param/AP_Param.h>

#define SLCAN_BUFFER_SIZE 200
#define SLCAN_RX_QUEUE_SIZE 64

#ifndef HAL_CAN_RX_STORAGE_SIZE
#define HAL_CAN_RX_QUEUE_SIZE 128
#endif

static_assert(HAL_CAN_RX_QUEUE_SIZE <= 254, "Invalid CAN Rx queue size");

namespace SLCAN {

class CANIface: public AP_HAL::CANIface {

    int16_t reportFrame(const AP_HAL::CanFrame& frame, uint64_t timestamp_usec);

    const char* processCommand(char* cmd);

    bool push_Frame(AP_HAL::CanFrame &frame);

    bool handle_FrameRTRStd(const char* cmd);
    bool handle_FrameRTRExt(const char* cmd);
    bool handle_FrameDataStd(const char* cmd);
    bool handle_FrameDataExt(const char* cmd);
    void slcan_passthrough_loop();
    inline void addByte(const uint8_t byte);

    bool initialized_;
    char buf_[SLCAN_BUFFER_SIZE + 1];
    unsigned _pending_frame_size = 0;
    int16_t pos_ = 0;
    AP_HAL::UARTDriver* _port;

    ObjectBuffer<AP_HAL::CANIface::CanRxItem> rx_queue_;

    const uint32_t _serial_lock_key = 0x53494442;
    AP_Int8 _slcan_can_port;
    AP_Int8 _slcan_ser_port;
    AP_Int8 _slcan_timeout;
    AP_Int8 _slcan_mode;

    AP_HAL::CANIface* _can_iface;
    HAL_Semaphore sem;
public:
    CANIface():
       rx_queue_(HAL_CAN_RX_QUEUE_SIZE)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    static const struct AP_Param::GroupInfo var_info[];

    bool init(const uint32_t bitrate, const OperatingMode mode) override { return false; }

    bool init_passthrough(uint8_t i);

    void flush() override { rx_queue_.clear(); }

    int set_port(AP_HAL::UARTDriver* port);

    bool is_initialized() const override;
    void reset_params();
    int16_t send(const AP_HAL::CanFrame& frame, uint64_t tx_deadline,
                         AP_HAL::CANIface::CanIOFlags flags) override;
    virtual int16_t receive(AP_HAL::CanFrame& out_frame, uint64_t& rx_time,
                             AP_HAL::CANIface::CanIOFlags& out_flags) override;
};

}

