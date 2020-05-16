/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Pavel Kirienko
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Siddharth Bharat Purohit
 */

#pragma once

#include "AP_HAL_ChibiOS.h"

#if MAX_NUMBER_OF_CAN_INTERFACES

#ifndef HAL_CAN_RX_STORAGE_SIZE
#define HAL_CAN_RX_QUEUE_SIZE 128
#endif

static_assert(HAL_CAN_RX_QUEUE_SIZE <= 254, "Invalid CAN Rx queue size");

/**
 * Single CAN iface.
 * The application shall not use this directly.
 */
class ChibiOS::CANDriver : public AP_HAL::CANDriver
{
    static constexpr unsigned long IDE = (0x40000000U); // Identifier Extension
    static constexpr unsigned long STID_MASK = (0x1FFC0000U); // Standard Identifier Mask
    static constexpr unsigned long EXID_MASK = (0x1FFFFFFFU); // Extended Identifier Mask
    static constexpr unsigned long RTR       = (0x20000000U); // Remote Transmission Request
    static constexpr unsigned long DLC_MASK  = (0x000F0000U); // Data Length Code

    /**
     * CANx register sets
     */
    typedef FDCAN_GlobalTypeDef CanType;
    
    struct CriticalSectionLocker {
        CriticalSectionLocker()
        {
            chSysSuspend();
        }
        ~CriticalSectionLocker()
        {
            chSysEnable();
        }
    };

    struct MessageRAM {
        uint32_t StandardFilterSA;
        uint32_t ExtendedFilterSA;
        uint32_t RxFIFO0SA;
        uint32_t RxFIFO1SA;
        uint32_t TxFIFOQSA;
        uint32_t EndAddress;
    } MessageRam_;

    struct Timings {
        uint16_t prescaler;
        uint8_t sjw;
        uint8_t bs1;
        uint8_t bs2;

        Timings()
            : prescaler(0)
            , sjw(0)
            , bs1(0)
            , bs2(0)
        { }
    };

    enum { NumTxMailboxes = 32 };

    static uint32_t FDCANMessageRAMOffset_;

    CanType* can_;
    uint64_t error_cnt_;
    uint32_t served_aborts_cnt_;
    ObjectBuffer<CanRxItem> rx_queue_;
    CanTxItem pending_tx_[NumTxMailboxes];
    uint8_t peak_tx_mailbox_index_;
    bool irq_init_;
    bool initialised_;
    bool had_activity_;
    ChibiOS::EventHandle* event_handle_;
    const uint8_t self_index_;

    int computeTimings(uint32_t target_bitrate, Timings& out_timings);

    void setupMessageRam(void);

    bool readRxFIFO(uint8_t fifo_index);

    void discardTimedOutTxMailboxes(uint64_t current_time);

    bool canAcceptNewTxFrame() const;

    bool isRxBufferEmpty() const;

    void pollErrorFlags();

    void checkAvailable(bool& read, bool& write, 
                            const AP_HAL::CanFrame* pending_tx) const;

    static uint32_t FDCAN2MessageRAMOffset_;
    static bool clock_init_;

public:
    /******************************************
     *   Common CAN methods                   *
     * ****************************************/
    CANDriver(uint8_t index);

    int init(const uint32_t bitrate, const OperatingMode mode) override;

    int16_t send(const AP_HAL::CanFrame& frame, uint64_t tx_deadline,
                 CanIOFlags flags) override;

    int16_t receive(AP_HAL::CanFrame& out_frame, uint64_t& out_timestamp_us,
                     CanIOFlags& out_flags) override;

    int16_t configureFilters(const CanFilterConfig* filter_configs,
            uint16_t num_configs) override;

    uint16_t getNumFilters() const override;

    uint64_t getErrorCount() const override;

    bool is_initialized() const override { return initialised_; }

    /******************************************
     * Select Method                          *
     * ****************************************/
    int16_t select(bool &read, bool &write,
                            const AP_HAL::CanFrame* pending_tx,
                            uint64_t blocking_deadline) override;

    bool set_event_handle(AP_HAL::EventHandle* handle) override;

    /************************************
     * Methods used inside interrupt    *
     ************************************/
    void handleTxCompleteInterrupt(uint64_t timestamp_us);

    void handleRxInterrupt(uint8_t fifo_index);
    void pollErrorFlagsFromISR(void);

    static constexpr CanType* const Can[MAX_NUMBER_OF_CAN_INTERFACES] = {
        reinterpret_cast<CanType*>(FDCAN1_BASE)
    #if MAX_NUMBER_OF_CAN_INTERFACES > 1
        ,
        reinterpret_cast<CanType*>(FDCAN2_BASE)
    #endif
    };
};


#endif //MAX_NUMBER_OF_CAN_INTERFACES
