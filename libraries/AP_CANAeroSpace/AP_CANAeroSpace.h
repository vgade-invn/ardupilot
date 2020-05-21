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
 * AP_CANAeroSpace.h
 */
 
#pragma once

#include <AP_HAL/CAN.h>
#include <AP_HAL/Semaphores.h>

#include <AP_Param/AP_Param.h>

#include <atomic>

class AP_CANAeroSpace : public AP_HAL::CANProtocol {
public:
    AP_CANAeroSpace();
    
    /* Do not allow copies */
    AP_CANAeroSpace(const AP_CANAeroSpace &other) = delete;
    AP_CANAeroSpace &operator=(const AP_CANAeroSpace&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    // Return CANAeroSpace from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_CANAeroSpace *get_CANAeroSpace(uint8_t driver_index);

    void init(uint8_t driver_index, bool enable_filters) override;

    void update();
    
private:
    void loop();

    bool _initialized;
    char _thread_name[11];
    uint8_t _driver_index;
    uavcan::ICanDriver* _can_driver;
    static const uint8_t CAN_IFACE_INDEX = 0;
};
