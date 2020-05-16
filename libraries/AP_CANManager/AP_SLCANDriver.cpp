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
 * Author: Siddharth Bharat Purohit
 * Referenced from implementation by Pavel Kirienko <pavel.kirienko@zubax.com>
 * for Zubax Babel
 */

#include <AP_HAL/AP_HAL.h>


#include "AP_SLCANDriver.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <stdio.h>
extern const AP_HAL::HAL& hal;

static uint8_t nibble2hex(uint8_t x)
{
    // Allocating in RAM because it's faster
    static uint8_t ConversionTable[] = {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
    };
    return ConversionTable[x & 0x0F];
}

static bool hex2nibble_error;

static uint8_t hex2nibble(char c)
{
    // Must go into RAM, not flash, because flash is slow
    static uint8_t NumConversionTable[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9
    };

    static uint8_t AlphaConversionTable[] = {
        10, 11, 12, 13, 14, 15
    };

    uint8_t out = 255;

    if (c >= '0' && c <= '9') {
        out = NumConversionTable[int(c) - int('0')];
    }
    else if (c >= 'a' && c <= 'f') {
        out = AlphaConversionTable[int(c) - int('a')];
    }
    else if (c >= 'A' && c <= 'F') {
        out = AlphaConversionTable[int(c) - int('A')];
    }

    if (out == 255) {
        hex2nibble_error = true;
    }
    return out;
}

int SLCAN::CANDriver::set_port(AP_HAL::UARTDriver* port) {
    if (port == nullptr) {
        return -1;
    }
    _port = port;
    return 0;
}


bool SLCAN::CANDriver::push_Frame(AP_HAL::CanFrame &frame)
{
    AP_HAL::CANDriver::CanRxItem frm;
    frm.frame = frame;
    frm.flags = 0;
    frm.timestamp_us = AP_HAL::micros64();
    return rx_queue_.push(frm);
}

/**
 * General frame format:
 *  <type> <id> <dlc> <data>
 * The emitting functions below are highly optimized for speed.
 */
bool SLCAN::CANDriver::handle_FrameDataExt(const char* cmd)
{
    AP_HAL::CanFrame f;
    hex2nibble_error = false;
    f.id = f.FlagEFF |
           (hex2nibble(cmd[1]) << 28) |
           (hex2nibble(cmd[2]) << 24) |
           (hex2nibble(cmd[3]) << 20) |
           (hex2nibble(cmd[4]) << 16) |
           (hex2nibble(cmd[5]) << 12) |
           (hex2nibble(cmd[6]) <<  8) |
           (hex2nibble(cmd[7]) <<  4) |
           (hex2nibble(cmd[8]) <<  0);
    if (cmd[9] < '0' || cmd[9] > ('0' + AP_HAL::CanFrame::MaxDataLen)) {
        return false;
    }
    f.dlc = cmd[9] - '0';
    if (f.dlc > AP_HAL::CanFrame::MaxDataLen) {
        return false;
    }
    {
        const char* p = &cmd[10];
        for (unsigned i = 0; i < f.dlc; i++) {
            f.data[i] = (hex2nibble(*p) << 4) | hex2nibble(*(p + 1));
            p += 2;
        }
    }
    if (hex2nibble_error) {
        return false;
    }
    return push_Frame(f);
}

bool SLCAN::CANDriver::handle_FrameDataStd(const char* cmd)
{
    AP_HAL::CanFrame f;
    hex2nibble_error = false;
    f.id = (hex2nibble(cmd[1]) << 8) |
           (hex2nibble(cmd[2]) << 4) |
           (hex2nibble(cmd[3]) << 0);
    if (cmd[4] < '0' || cmd[4] > ('0' + AP_HAL::CanFrame::MaxDataLen)) {
        return false;
    }
    f.dlc = cmd[4] - '0';
    if (f.dlc > AP_HAL::CanFrame::MaxDataLen) {
        return false;
    }
    {
        const char* p = &cmd[5];
        for (unsigned i = 0; i < f.dlc; i++) {
            f.data[i] = (hex2nibble(*p) << 4) | hex2nibble(*(p + 1));
            p += 2;
        }
    }
    if (hex2nibble_error) {
        return false;
    }
    return push_Frame(f);
}

bool SLCAN::CANDriver::handle_FrameRTRExt(const char* cmd)
{
    AP_HAL::CanFrame f;
    hex2nibble_error = false;
    f.id = f.FlagEFF | f.FlagRTR |
           (hex2nibble(cmd[1]) << 28) |
           (hex2nibble(cmd[2]) << 24) |
           (hex2nibble(cmd[3]) << 20) |
           (hex2nibble(cmd[4]) << 16) |
           (hex2nibble(cmd[5]) << 12) |
           (hex2nibble(cmd[6]) <<  8) |
           (hex2nibble(cmd[7]) <<  4) |
           (hex2nibble(cmd[8]) <<  0);
    if (cmd[9] < '0' || cmd[9] > ('0' + AP_HAL::CanFrame::MaxDataLen)) {
        return false;
    }
    f.dlc = cmd[9] - '0';

    if (f.dlc > AP_HAL::CanFrame::MaxDataLen) {
        return false;
    }
    if (hex2nibble_error) {
        return false;
    }
    return push_Frame(f);
}

bool SLCAN::CANDriver::handle_FrameRTRStd(const char* cmd)
{
    AP_HAL::CanFrame f;
    hex2nibble_error = false;
    f.id = f.FlagRTR |
           (hex2nibble(cmd[1]) << 8) |
           (hex2nibble(cmd[2]) << 4) |
           (hex2nibble(cmd[3]) << 0);
    if (cmd[4] < '0' || cmd[4] > ('0' + AP_HAL::CanFrame::MaxDataLen)) {
        return false;
    }
    f.dlc = cmd[4] - '0';
    if (f.dlc <= AP_HAL::CanFrame::MaxDataLen) {
        return false;
    }
    if (hex2nibble_error) {
        return false;
    }
    return push_Frame(f);
}

static inline const char* getASCIIStatusCode(bool status)
{
    return status ? "\r" : "\a";
}


// bool SLCAN::CANManager::begin(uint32_t bitrate, uint8_t can_number)
// {
//     if (driver_.init(bitrate, SLCAN::CANDriver::NormalMode, nullptr) < 0) {
//         return false;
//     }
//     if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&SLCAN::CANManager::reader_trampoline, void), "SLCAN", 4096, AP_HAL::Scheduler::PRIORITY_CAN, -1)) {
//         return false;
//     }
//     initialized(true);
//     return true;
// }

bool SLCAN::CANDriver::is_initialized() const
{
    return initialized_;
}

// void SLCAN::CANManager::initialized(bool val)
// {
//     initialized_ = val;
// }

int SLCAN::CANDriver::init()
{
    if (_port == nullptr) {
        return -1;
    }
    _port->lock_port(_serial_lock_key, _serial_lock_key);
    initialized_ = true;
    return 0;
}

/**
 * General frame format:
 *  <type> <id> <dlc> <data> [timestamp msec] [flags]
 * Types:
 *  R - RTR extended
 *  r - RTR standard
 *  T - Data extended
 *  t - Data standard
 * Flags:
 *  L - this frame is a loopback frame; timestamp field contains TX timestamp
 */
int16_t SLCAN::CANDriver::reportFrame(const AP_HAL::CanFrame& frame, bool loopback, uint64_t timestamp_usec)
{
    if (_port == nullptr) {
        return -1;
    }
    constexpr unsigned SLCANMaxFrameSize = 40;
    uint8_t buffer[SLCANMaxFrameSize] = {'\0'};
    uint8_t* p = &buffer[0];
    /*
        * Frame type
        */
    if (frame.isRemoteTransmissionRequest()) {
        *p++ = frame.isExtended() ? 'R' : 'r';
    }
    else if (frame.isErrorFrame()) {
        return -1;     // Not supported
    }
    else {
        *p++ = frame.isExtended() ? 'T' : 't';
    }

    /*
        * ID
        */
    {
        const uint32_t id = frame.id & frame.MaskExtID;
        if (frame.isExtended()) {
            *p++ = nibble2hex(id >> 28);
            *p++ = nibble2hex(id >> 24);
            *p++ = nibble2hex(id >> 20);
            *p++ = nibble2hex(id >> 16);
            *p++ = nibble2hex(id >> 12);
        }
        *p++ = nibble2hex(id >> 8);
        *p++ = nibble2hex(id >> 4);
        *p++ = nibble2hex(id >> 0);
    }

    /*
        * DLC
        */
    *p++ = char('0' + frame.dlc);

    /*
        * Data
        */
    for (unsigned i = 0; i < frame.dlc; i++) {
        const uint8_t byte = frame.data[i];
        *p++ = nibble2hex(byte >> 4);
        *p++ = nibble2hex(byte);
    }

    /*
        * Timestamp
        */
    //if (param_cache.timestamping_on)
    {
        // SLCAN format - [0, 60000) milliseconds
        const auto slcan_timestamp = uint16_t(timestamp_usec / 1000U);
        *p++ = nibble2hex(slcan_timestamp >> 12);
        *p++ = nibble2hex(slcan_timestamp >> 8);
        *p++ = nibble2hex(slcan_timestamp >> 4);
        *p++ = nibble2hex(slcan_timestamp >> 0);
    }

    /*
        * Flags
        */
    //if (param_cache.flags_on)
    {
        if (loopback) {
            *p++ = 'L';
        }
    }

    /*
        * Finalization
        */
    *p++ = '\r';
    const auto frame_size = unsigned(p - &buffer[0]);

    if (_port->txspace() < _pending_frame_size) {
        _pending_frame_size = frame_size;
        return 0;
    }
    //Write to Serial
    if (!_port->write_locked(&buffer[0], frame_size, _serial_lock_key)) {
        return 0;
    }
    return 1;
}


/**
 * Accepts command string, returns response string or nullptr if no response is needed.
 */
const char* SLCAN::CANDriver::processCommand(char* cmd)
{

    if (_port == nullptr) {
        return nullptr;
    }

    /*
        * High-traffic SLCAN commands go first
        */
    if (cmd[0] == 'T') {
        return handle_FrameDataExt(cmd) ? "Z\r" : "\a";
    }
    else if (cmd[0] == 't') {
        return handle_FrameDataStd(cmd) ? "z\r" : "\a";
    }
    else if (cmd[0] == 'R') {
        return handle_FrameRTRExt(cmd) ? "Z\r" : "\a";
    }
    else if (cmd[0] == 'r' && cmd[1] <= '9') { // The second condition is needed to avoid greedy matching
        // See long commands below
        return handle_FrameRTRStd(cmd) ? "z\r" : "\a";
    }
    uint8_t resp_bytes[40];
    uint16_t resp_len;
    /*
        * Regular SLCAN commands
        */
    switch (cmd[0]) {
    case 'S':               // Set CAN bitrate
    case 'O':               // Open CAN in normal mode
    case 'L':               // Open CAN in listen-only mode
    case 'l':               // Open CAN with loopback enabled
    case 'C':               // Close CAN
    case 'M':               // Set CAN acceptance filter ID
    case 'm':               // Set CAN acceptance filter mask
    case 'U':               // Set UART baud rate, see http://www.can232.com/docs/can232_v3.pdf
    case 'Z': {             // Enable/disable RX and loopback timestamping
        return getASCIIStatusCode(true);    // Returning success for compatibility reasons
    }
    case 'F': {             // Get status flags
        resp_len = snprintf((char*)resp_bytes, sizeof(resp_bytes), "F%02X\r", unsigned(0));    // Returning success for compatibility reasons
        if (resp_len > 0) {
            _port->write_locked(resp_bytes, resp_len, _serial_lock_key);
        }
        return nullptr;
    }
    case 'V': {             // HW/SW version
        resp_len = snprintf((char*)resp_bytes, sizeof(resp_bytes),"V%x%x%x%x\r", 1, 0, 1, 0);
        if (resp_len > 0) {
            _port->write_locked(resp_bytes, resp_len, _serial_lock_key);
        }
        return nullptr;
    }
    case 'N': {             // Serial number
        const uint8_t uid_buf_len = 12;
        uint8_t uid_len = uid_buf_len;
        uint8_t unique_id[uid_buf_len];
        char buf[uid_buf_len * 2 + 1] = {'\0'};
        char* pos = &buf[0];
        if (hal.util->get_system_id_unformatted(unique_id, uid_len)) {
            for (uint8_t i = 0; i < uid_buf_len; i++) {
                *pos++ = nibble2hex(unique_id[i] >> 4);
                *pos++ = nibble2hex(unique_id[i]);
            }
        }
        *pos++ = '\0';
        resp_len = snprintf((char*)resp_bytes, sizeof(resp_bytes),"N%s\r", &buf[0]);
        if (resp_len > 0) {
            _port->write_locked(resp_bytes, resp_len, _serial_lock_key);
        }
        return nullptr;
    }
    default: {
        break;
    }
    }

    return getASCIIStatusCode(false);
}

/**
 * Please keep in mind that this function is strongly optimized for speed.
 */
inline void SLCAN::CANDriver::addByte(const uint8_t byte)
{
    if (_port == nullptr) {
        return;
    }
    if ((byte >= 32 && byte <= 126)) {                // Normal printable ASCII character
        if (pos_ < SLCAN_BUFFER_SIZE) {
            buf_[pos_] = char(byte);
            pos_ += 1;
        }
        else {
            pos_ = 0;                                        // Buffer overrun; silently drop the data
        }
    }
    else if (byte == '\r') {                          // End of command (SLCAN)

        // Processing the command
        buf_[pos_] = '\0';
        const char* const response = processCommand(reinterpret_cast<char*>(&buf_[0]));
        pos_ = 0;

        // Sending the response if provided
        if (response != nullptr) {
            
            _port->write_locked(reinterpret_cast<const uint8_t*>(response),
                                strlen(response), _serial_lock_key);
        }
    }
    else if (byte == 8 || byte == 127) {            // DEL or BS (backspace)
        if (pos_ > 0) {
            pos_ -= 1;
        }
    }
    else {                                                  // This also includes Ctrl+C, Ctrl+D
        pos_ = 0;                                            // Invalid byte - drop the current command
    }
}

// void SLCAN::CANDriver::reader()
// {
//     if (_port == nullptr) {
//         return;
//     }
//     if (!_port_initialised) {
//         //_port->begin(bitrate_);
//         _port_initialised = true;
//     }
//     _port->lock_port(_serial_lock_key, _serial_lock_key);
//     if (!_port->wait_timeout(1,1)) {
//         int16_t data = _port->read_locked(_serial_lock_key);
//         while (data > 0) {
//             addByte(data);
//             data = _port->read_locked(_serial_lock_key);
//         }
//     }
// }

int16_t SLCAN::CANDriver::send(const AP_HAL::CanFrame& frame, uint64_t tx_deadline, AP_HAL::CANDriver::CanIOFlags flags)
{
    if (frame.isErrorFrame() || frame.dlc > 8) {
        return -ErrUnsupportedFrame;
    }

    return reportFrame(frame, flags & AP_HAL::CANDriver::CanIOFlagLoopback, AP_HAL::micros64());
}

int16_t SLCAN::CANDriver::receive(AP_HAL::CanFrame& out_frame, uint64_t& rx_time,
                                  AP_HAL::CANDriver::CanIOFlags& out_flags)
{
    if (_port == nullptr) {
        return 0;
    }
    if (!_port->available_locked(_serial_lock_key)) {
        return 0;
    } else {
        uint32_t num_bytes = _port->available_locked(_serial_lock_key);
        while(num_bytes--) {
            int16_t ret = _port->read_locked(_serial_lock_key);
            if (ret <= 0) {
                break;
            }
            addByte(ret);
        }
    }
    if (!rx_queue_.available()) {
        return 0;
    }
    CanRxItem frm;
    rx_queue_.pop(frm);
    out_frame = frm.frame;
    rx_time = frm.timestamp_us;
    out_flags = frm.flags;
    return 1;
}

// bool SLCAN::CANDriver::pending_frame_sent()
// {
//     if (_pending_frame_size == 0) {
//         return false;
//     }
//     else if (_port->txspace() >= _pending_frame_size) {
//         _pending_frame_size = 0;
//         return true;
//     }
//     return false;
// }

// bool SLCAN::CANDriver::isRxBufferEmpty()
// {
//     return rx_queue_.available() == 0;
// }

// bool SLCAN::CANDriver::canAcceptNewTxFrame() const
// {
//     constexpr unsigned SLCANMaxFrameSize = 40;
//     if (_port->txspace() >= SLCANMaxFrameSize) {
//         return true;
//     }
//     return false;
// }

// uavcan::CanSelectMasks SLCAN::CANManager::makeSelectMasks(const uavcan::CanFrame* (&pending_tx)[uavcan::MaxCanIfaces])
// {
//     uavcan::CanSelectMasks msk;

//     for (uint8_t i = 0; i < _ifaces_num; i++) {
//         if (!driver_.is_initialized()) {
//             continue;
//         }

//         if (!driver_.isRxBufferEmpty()) {
//             msk.read |= 1 << i;
//         }

//         if (pending_tx[i] != nullptr) {
//             if (driver_.canAcceptNewTxFrame()) {
//                 msk.write |= 1 << i;
//             }
//         }
//     }

//     return msk;
// }

// int16_t SLCAN::CANManager::select(uavcan::CanSelectMasks& inout_masks,
//                                   const uavcan::CanFrame* (&pending_tx)[uavcan::MaxCanIfaces], uavcan::MonotonicTime blocking_deadline)
// {
//     const uavcan::CanSelectMasks in_masks = inout_masks;
//     const uavcan::MonotonicTime time = uavcan::MonotonicTime::fromUSec(AP_HAL::micros64());

//     inout_masks = makeSelectMasks(pending_tx); // Check if we already have some of the requested events
//     if ((inout_masks.read & in_masks.read) != 0 || (inout_masks.write & in_masks.write) != 0) {
//         return 1;
//     }
//     _irq_handler_ctx = chThdGetSelfX();
//     if (blocking_deadline.toUSec()) {
//         chEvtWaitAnyTimeout(ALL_EVENTS, chTimeUS2I((blocking_deadline - time).toUSec())); // Block until timeout expires or any iface updates
//     }
//     inout_masks = makeSelectMasks(pending_tx); // Return what we got even if none of the requested events are set
//     return 1; // Return value doesn't matter as long as it is non-negative
// }

// void SLCAN::CANManager::reader_trampoline(void)
// {
//     while (true) {
//         driver_.reader();
//         if ((driver_.pending_frame_sent() || !driver_.isRxBufferEmpty()) && _irq_handler_ctx) {
//             chEvtSignalI(_irq_handler_ctx, EVENT_MASK(0));
//         }
//     }
// }
