
#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_HAL/utility/RingBuffer.h"

#define SLCAN_BUFFER_SIZE 200
#define SLCAN_RX_QUEUE_SIZE 64

#ifndef HAL_CAN_RX_STORAGE_SIZE
#define HAL_CAN_RX_QUEUE_SIZE 128
#endif

static_assert(HAL_CAN_RX_QUEUE_SIZE <= 254, "Invalid CAN Rx queue size");

namespace SLCAN {

class CANDriver: public AP_HAL::CANDriver {

    int16_t reportFrame(const AP_HAL::CanFrame& frame, bool loopback, uint64_t timestamp_usec);

    const char* processCommand(char* cmd);

    bool push_Frame(AP_HAL::CanFrame &frame);

    bool handle_FrameRTRStd(const char* cmd);
    bool handle_FrameRTRExt(const char* cmd);
    bool handle_FrameDataStd(const char* cmd);
    bool handle_FrameDataExt(const char* cmd);

    inline void addByte(const uint8_t byte);

    bool initialized_;
    char buf_[SLCAN_BUFFER_SIZE + 1];
    unsigned _pending_frame_size = 0;
    int16_t pos_ = 0;
    AP_HAL::UARTDriver* _port;

    ObjectBuffer<AP_HAL::CANDriver::CanRxItem> rx_queue_;

    HAL_Semaphore rx_sem_;

    const uint32_t _serial_lock_key = 0x53494442;

public:
    CANDriver():
       rx_queue_(HAL_CAN_RX_QUEUE_SIZE)
    {}

    int init(const uint32_t bitrate, const OperatingMode mode) override { return 0; }

    int init();

    int set_port(AP_HAL::UARTDriver* port);

    bool is_initialized() const override;

    int16_t send(const AP_HAL::CanFrame& frame, uint64_t tx_deadline,
                         AP_HAL::CANDriver::CanIOFlags flags) override;
    virtual int16_t receive(AP_HAL::CanFrame& out_frame, uint64_t& rx_time,
                             AP_HAL::CANDriver::CanIOFlags& out_flags) override;

};

}

