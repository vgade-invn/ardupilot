/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * AP_CANAeroSpace.cpp
 *
 *      Author: Francisco Ferreira
 */

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

#include <AP_Common/AP_Common.h>

#include <AP_HAL/utility/sparse-endian.h>

#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>

#include "AP_CANAeroSpace.h"

extern const AP_HAL::HAL& hal;

#define debug_can(level_debug, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(_driver_index)) { printf(fmt, ##args); }} while (0)

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_CANAeroSpace::var_info[] = {

    AP_GROUPEND
};

AP_CANAeroSpace::AP_CANAeroSpace()
{
    debug_can(2, "CANAeroSpace: constructed\n\r");
}

AP_CANAeroSpace *AP_CANAeroSpace::get_CANAeroSpace(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_protocol_type(driver_index) != AP_BoardConfig_CAN::Protocol_Type_CANAeroSpace) {
        return nullptr;
    }
    return static_cast<AP_CANAeroSpace*>(AP::can().get_driver(driver_index));
}

void AP_CANAeroSpace::init(uint8_t driver_index, bool enable_filters)
{
    _driver_index = driver_index;

    debug_can(2, "CANAeroSpace: starting init\n\r");

    if (_initialized) {
        debug_can(1, "CANAeroSpace: already initialized\n\r");
        return;
    }

    // get CAN manager instance
    AP_HAL::CANManager* can_mgr = hal.can_mgr[driver_index];

    if (can_mgr == nullptr) {
        debug_can(1, "CANAeroSpace: no mgr for this driver\n\r");
        return;
    }

    if (!can_mgr->is_initialized()) {
        debug_can(1, "CANAeroSpace: mgr not initialized\n\r");
        return;
    }

    // store pointer to CAN driver
    _can_driver = can_mgr->get_driver();

    if (_can_driver == nullptr) {
        debug_can(1, "CANAeroSpace: no CAN driver\n\r");
        return;
    }

    snprintf(_thread_name, sizeof(_thread_name), "CAS_%u", driver_index);

    // start thread for receiving and sending CAN frames
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_CANAeroSpace::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        debug_can(1, "CANAeroSpace: couldn't create thread\n\r");
        return;
    }

    _initialized = true;

    debug_can(2, "CANAeroSpace: init done\n\r");

    return;
}

void AP_CANAeroSpace::loop()
{
    uavcan::MonotonicTime timeout;
    uavcan::CanFrame empty_frame { (0 | uavcan::CanFrame::FlagEFF), nullptr, 0 };
    const uavcan::CanFrame* select_frames[uavcan::MaxCanIfaces] { };
    select_frames[CAN_IFACE_INDEX] = &empty_frame;

    const uint32_t LOOP_INTERVAL_US = 1000;

    while (true) {
        if (!_initialized) {
            debug_can(2, "CANAeroSpace: not initialized\n\r");
            hal.scheduler->delay_microseconds(2000);
            continue;
        }

        uavcan::CanSelectMasks inout_mask;
        uint64_t now = AP_HAL::micros64();

        // always look for received frames
        inout_mask.read = 1 << CAN_IFACE_INDEX;
        timeout = uavcan::MonotonicTime::fromUSec(now + LOOP_INTERVAL_US);

        // wait for write space or receive frame
        uavcan::CanSelectMasks in_mask = inout_mask;
        _can_driver->select(inout_mask, select_frames, timeout);

        if (in_mask.read & inout_mask.read) {
            uavcan::CanFrame frame;
            uavcan::MonotonicTime time;
            uavcan::UtcTime utc_time;
            uavcan::CanIOFlags flags {};

            int16_t res = _can_driver->getIface(CAN_IFACE_INDEX)->receive(frame, time, utc_time, flags);

            if (res == 1) {
                ::printf("got frame id=0x%x dlc=%u [%02x %02x %02x %02x %02x %02x %02x %02x]\n",
                         frame.id, frame.dlc,
                         frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                         frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
            }
        }
    }
}

void AP_CANAeroSpace::update()
{
}


#endif // HAL_WITH_UAVCAN
