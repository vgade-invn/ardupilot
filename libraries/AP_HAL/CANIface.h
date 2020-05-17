/*
 * Copyright (C) 2020 Siddharth B Purohit
 *
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
 */

#pragma once

#include <stdint.h>
#include "AP_HAL_Namespace.h"

/**
 * Raw CAN frame, as passed to/from the CAN driver.
 */
struct AP_HAL::CanFrame
{
    static const uint32_t MaskStdID = 0x000007FFU;
    static const uint32_t MaskExtID = 0x1FFFFFFFU;
    static const uint32_t FlagEFF = 1U << 31;                  ///< Extended frame format
    static const uint32_t FlagRTR = 1U << 30;                  ///< Remote transmission request
    static const uint32_t FlagERR = 1U << 29;                  ///< Error frame

    static const uint8_t MaxDataLen = 8;

    uint32_t id;                ///< CAN ID with flags (above)
    uint8_t data[MaxDataLen];
    uint8_t dlc;                ///< Data Length Code

    CanFrame() :
        id(0),
        dlc(0)
    {
        memset(data,0, MaxDataLen);
    }

    CanFrame(uint32_t can_id, const uint8_t* can_data, uint8_t data_len) :
        id(can_id),
        dlc((data_len > MaxDataLen) ? MaxDataLen : data_len)
    {
        if ((can_data == nullptr) || (data_len != dlc)) {
            id = 0;
            dlc = 0;
        }
        memcpy(this->data, can_data, dlc);
    }

    bool operator!=(const CanFrame& rhs) const { return !operator==(rhs); }
    bool operator==(const CanFrame& rhs) const
    {
        return (id == rhs.id) && (dlc == rhs.dlc) && (memcmp(data, rhs.data, dlc) == 0);
    }

    bool isExtended()                  const { return id & FlagEFF; }
    bool isRemoteTransmissionRequest() const { return id & FlagRTR; }
    bool isErrorFrame()                const { return id & FlagERR; }

    /**
     * CAN frame arbitration rules, particularly STD vs EXT:
     *     Marco Di Natale - "Understanding and using the Controller Area Network"
     *     http://www6.in.tum.de/pub/Main/TeachingWs2013MSE/CANbus.pdf
     */
    bool priorityHigherThan(const CanFrame& rhs) const;
    bool priorityLowerThan(const CanFrame& rhs) const { return rhs.priorityHigherThan(*this); }
};

/**
 * Single non-blocking CAN interface.
 */
class AP_HAL::CANIface {
public:
    /* Driver error codes.
    * These values can be returned from driver functions negated.
    */
    enum OperatingMode {
        PassThroughMode,
        NormalMode,
        SilentMode
    };

    typedef uint16_t CanIOFlags;
    static const CanIOFlags Loopback = 1;
    static const CanIOFlags AbortOnError = 2;

    struct CanRxItem {
        uint64_t timestamp_us;
        CanIOFlags flags;
        CanFrame frame;
    };

    struct CanTxItem {
        uint64_t deadline;
        CanFrame frame;
        bool loopback;
        bool abort_on_error;
        bool aborted;
        bool pushed;
        bool setup;
        uint8_t index;
    };

    struct CanFilterConfig
    {
        uint32_t id;
        uint32_t mask;

        bool operator==(const CanFilterConfig& rhs) const
        {
            return rhs.id == id && rhs.mask == mask;
        }
    };

    virtual bool init(const uint32_t bitrate, const OperatingMode mode) = 0;
    
    /******************************************
     * Select method                          *
     * ****************************************/
    virtual bool select(bool &read_select, bool &write_select, 
                        const CanFrame* const pending_tx, uint64_t timeout) { return false; }
    
    virtual bool set_event_handle(EventHandle* evt_handle) { return true; }
    
    /**
     * Non-blocking transmission.
     *
     * If the frame wasn't transmitted upon TX deadline, the driver should discard it.
     *
     * Note that it is LIKELY that the library will want to send the frames that were passed into the select()
     * method as the next ones to transmit, but it is NOT guaranteed. The library can replace those with new
     * frames between the calls.
     *
     * @return 1 = one frame transmitted, 0 = TX buffer full, negative for error.
     */
    virtual int16_t send(const CanFrame& frame, uint64_t tx_deadline, CanIOFlags flags) = 0;

    /**
     * Non-blocking reception.
     *
     * Timestamps should be provided by the CAN driver, ideally by the hardware CAN controller.
     *
     * Monotonic timestamp is required and can be not precise since it is needed only for
     * protocol timing validation (transfer timeouts and inter-transfer intervals).
     *
     * UTC timestamp is optional, if available it will be used for precise time synchronization;
     * must be set to zero if not available.
     *
     * Refer to @ref ISystemClock to learn more about timestamps.
     *
     * @param [out] out_ts_monotonic Monotonic timestamp, mandatory.
     * @return 1 = one frame received, 0 = RX buffer empty, negative for error.
     */
    virtual int16_t receive(CanFrame& out_frame, uint64_t& out_ts_monotonic, CanIOFlags& out_flags) = 0;

    /**
     * Configure the hardware CAN filters. @ref CanFilterConfig.
     *
     * @return 0 = success, negative for error.
     */
    virtual bool configureFilters(const CanFilterConfig* filter_configs, uint16_t num_configs) { return 0; }

    /**
     * Number of available hardware filters.
     */
    virtual uint16_t getNumFilters() const { return 0; }

    /**
     * Continuously incrementing counter of hardware errors.
     * Arbitration lost should not be treated as a hardware error.
     */
    virtual uint32_t getErrorCount() const { return 0; }

    virtual uint32_t get_stats(char* data, uint32_t max_size) { return 0; }

    virtual bool is_busoff() const { return false; }

    virtual void flush() = 0;

    virtual bool is_initialized() const = 0;
};
