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
 *   AP_BoardConfig_CAN - board specific configuration for CAN interface
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include "AP_CANManager.h"

#if MAX_NUMBER_OF_CAN_INTERFACES

#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#include <AP_KDECAN/AP_KDECAN.h>
#include <AP_ToshibaCAN/AP_ToshibaCAN.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_PiccoloCAN/AP_PiccoloCAN.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux/CAN.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_HAL_ChibiOS/CANDriver.h>
#endif

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_CANManager::var_info[] = {

#if MAX_NUMBER_OF_CAN_INTERFACES > 0
    // @Group: P1_
    // @Path: ../AP_BoardConfig/canbus_interface.cpp
    AP_SUBGROUPINFO(_interfaces[0], "P1_", 1, AP_CANManager, AP_CANManager::Interface),
#endif

#if MAX_NUMBER_OF_CAN_INTERFACES > 1
    // @Group: P2_
    // @Path: ../AP_BoardConfig/canbus_interface.cpp
    AP_SUBGROUPINFO(_interfaces[1], "P2_", 2, AP_CANManager, AP_CANManager::Interface),
#endif

#if MAX_NUMBER_OF_CAN_INTERFACES > 2
    // @Group: P3_
    // @Path: ../AP_BoardConfig/canbus_interface.cpp
    AP_SUBGROUPINFO(_interfaces[2], "P3_", 3, AP_CANManager, AP_CANManager::Interface),
#endif

#if MAX_NUMBER_OF_CAN_INTERFACES > 0
    // @Group: D1_
    // @Path: ../AP_BoardConfig/canbus_driver.cpp
    AP_SUBGROUPINFO(_protocols[0], "D1_", 4, AP_CANManager, AP_CANManager::Protocol),
#endif

#if MAX_NUMBER_OF_CAN_INTERFACES > 1
    // @Group: D2_
    // @Path: ../AP_BoardConfig/canbus_driver.cpp
    AP_SUBGROUPINFO(_protocols[1], "D2_", 5, AP_CANManager, AP_CANManager::Protocol),
#endif

#if MAX_NUMBER_OF_CAN_INTERFACES > 2
    // @Group: D3_
    // @Path: ../AP_BoardConfig/canbus_driver.cpp
    AP_SUBGROUPINFO(_protocols[2], "D3_", 6, AP_CANManager, AP_CANManager::Protocol),
#endif
    
    // @Param: CPORT
    // @DisplayName: SLCAN Route
    // @Description: CAN Driver ID to be routed to SLCAN, 0 means no routing
    // @Values: 0:Disabled,1:First driver,2:Second driver
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("SLCAN_CPORT", 7, AP_CANManager, _slcan_can_port, 0),

    // @Param: SERNUM
    // @DisplayName: SLCAN Serial Port
    // @Description: Serial Port ID to be used for temporary SLCAN iface, -1 means no temporary serial. This parameter is automatically reset on reboot or on timeout. See CAN_SLCAN_TIMOUT for timeout details
    // @Values: -1:Disabled,0:Serial0,1:Serial1,2:Serial2,3:Serial3,4:Serial4,5:Serial5,6:Serial6
    // @User: Standard
    AP_GROUPINFO("SLCAN_SERNUM", 8, AP_CANManager, _slcan_ser_port, -1),

    // @Param: TIMOUT
    // @DisplayName: SLCAN Timeout
    // @Description: Duration of inactivity after which SLCAN is switched back to original protocol in seconds.
    // @Range: 0 32767
    // @User: Standard
    AP_GROUPINFO("SLCAN_TIMOUT", 9, AP_CANManager, _slcan_timeout, 0),

    // @Param: MODE
    // @DisplayName: SLCAN MODE
    // @Description: SLCAN Mode to be selected, Passthrough mode routes all can drivers to and from SLCAN. Interface Mode puts selected protocol on SLCAN instead of CAN bus, and Silent mode routes all packets on the selected CAN Bus
    // @Range: 0 2
    // @Values: 0: Passthrough Mode 1: Interface Mode 2: Silent Mode
    // @User: Standard
    AP_GROUPINFO("SLCAN_MODE", 10, AP_CANManager, _slcan_mode, 0),

    AP_GROUPEND
};

AP_CANManager *AP_CANManager::_singleton;

AP_CANManager::AP_CANManager()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_CANManager must be singleton");
    }
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _singleton = this;
}

void AP_CANManager::init()
{
    // Create all drivers that we need
    _slcan_ser_port.set_and_save(-1);
    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_INTERFACES; i++) {
        uint8_t protocol_number = _interfaces[i]._protocol_number_cache = _interfaces[i]._protocol_number;
        if (protocol_number == 0) {
            continue;
        }

        // Check the driver number assigned to this physical interface
        if (hal.can[i] == nullptr) {
            // So if this driver was not created before for other physical interface - do it
            #if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
                const_cast <AP_HAL::HAL&> (hal).can[i] = new Linux::CANDriver;
            #elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
                const_cast <AP_HAL::HAL&> (hal).can[i] = new ChibiOS::CANDriver(i);
            #endif
        }

        // For this now existing driver, start the physical interface
        if (hal.can[i] != nullptr) {
            hal.can[i]->init(_interfaces[i]._bitrate, AP_HAL::CANDriver::NormalMode);
        } else {
            continue;
        }

        //Setup SLCAN related config
        if (_slcan_mode == AP_HAL::CANDriver::PassThroughMode && _slcan_can_port == (i+1) 
            && hal.can[i]->is_initialized()) {
            // We will not be running any protocol on this port, 
            // just forward everything we receive on this port
            hal.console->printf("CANManager: Launching SLCAN Passthrough thread for CAN%d\n", _slcan_can_port - 1);
            if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_CANManager::slcan_passthrough_loop, void), "SLCAN_Passthrough", 1024, AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
                hal.console->printf("Unable to launch slcan passthrough thread\n");
            }
            continue;
        }

        int p = protocol_number - 1;
        Protocol_Type prot_type = _protocols[p]._protocol_type_cache = (Protocol_Type) _protocols[p]._protocol_type.get();

        hal.console->printf("can driver %d initialized well\n\r", i + 1);

        if (_protocols[p]._protocol != nullptr) {
            //We already initialised the driver just add interface and move on
            hal.console->printf("Adding Interface %d to Driver %d\n\r", i + 1, p + 1);
            _protocols[p]._protocol->add_interface(hal.can[i]);
            continue;
        }

        _num_protocols++;
        
        if (prot_type == Protocol_Type_UAVCAN) {
            _protocols[p]._protocol = _protocols[p]._uavcan =  new AP_UAVCAN;

            if (_protocols[p]._protocol == nullptr) {
                AP_HAL::panic("Failed to allocate uavcan %d\n\r", i + 1);
                continue;
            }

            AP_Param::load_object_from_eeprom(_protocols[p]._uavcan, AP_UAVCAN::var_info);
        } else if (prot_type == Protocol_Type_KDECAN) {
// To be replaced with macro saying if KDECAN library is included
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter) || APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_TYPE(APM_BUILD_ArduSub)
            _protocols[p]._protocol = _protocols[p]._kdecan =  new AP_KDECAN;

                if (_protocols[p]._protocol == nullptr) {
                AP_HAL::panic("Failed to allocate KDECAN %d\n\r", p + 1);
                continue;
            }

            AP_Param::load_object_from_eeprom(_protocols[p]._kdecan, AP_KDECAN::var_info);
#endif
        } else if (prot_type == Protocol_Type_ToshibaCAN) {
            _protocols[p]._protocol = _protocols[p]._tcan = new AP_ToshibaCAN;

            if (_protocols[p]._protocol == nullptr) {
                AP_BoardConfig::config_error("ToshibaCAN init failed");
                continue;
            }
#if HAL_PICCOLO_CAN_ENABLE
        } else if (prot_type == Protocol_Type_PiccoloCAN) {
            _protocols[p]._protocol = _protocols[p]._pcan = new AP_PiccoloCAN;

            if (_protocols[p]._protocol == nullptr) {
                AP_BoardConfig::config_error("PiccoloCAN init failed");
                continue;
            }
#endif
        } else {
            continue;
        }

        _protocols[p]._protocol->add_interface(hal.can[i]);
    }

    for (uint8_t p = 0; p < MAX_NUMBER_OF_CAN_PROTOCOLS; p++) {
        //initialise all the drivers
        if (_protocols[p]._protocol == nullptr) {
            continue;
        }
        _protocols[p]._protocol->init(p, true);
    }

    // param count could have changed
    AP_Param::invalidate_count();
}

void AP_CANManager::slcan_passthrough_loop() {
    uint8_t drv_num = _slcan_can_port - 1; 
    AP_HAL::CanFrame frame;
    bool read_select;
    bool write_select;
    uint64_t timestamp_us;
    AP_HAL::CANDriver::CanIOFlags flags;
    uint8_t prev_ser_port = 255;
    int8_t _slcan_ser_port_cache;
    while(true) {
        _slcan_ser_port_cache = _slcan_ser_port;
        while (_slcan_ser_port_cache < 0) {
            hal.scheduler->delay(100);
            _slcan_ser_port_cache = _slcan_ser_port;
        }

        if (prev_ser_port != _slcan_ser_port_cache) {
            AP_HAL::UARTDriver* slcan_serial = AP::serialmanager().get_serial_by_id(_slcan_ser_port_cache);
            slcan_driver.set_port(slcan_serial);
            prev_ser_port = _slcan_ser_port_cache;
            slcan_driver.init();
            gcs().send_text(MAV_SEVERITY_INFO, "CANManager: Starting SLCAN Passthrough on Serial %d with CAN%d", _slcan_ser_port_cache, drv_num);
            if (!slcan_driver.is_initialized()) {
                _slcan_ser_port.set_and_save(-1);
                _slcan_ser_port_cache = -1;
                continue;
            }
        }
        read_select = true;
        write_select = true;
        hal.can[drv_num]->select(read_select, write_select, nullptr, 0);
        if (read_select) {
            //read data from can and put on slcan
            if (hal.can[drv_num]->receive(frame, timestamp_us, flags) > 0) {
                slcan_driver.send(frame, AP_HAL::micros() + 1000, flags);
            }
        }
        //read data from slcan and put on can
        if (slcan_driver.receive(frame, timestamp_us, flags) > 0) {
            hal.can[drv_num]->send(frame, AP_HAL::micros() + 1000, flags);
        }
        hal.scheduler->delay_microseconds(1000);
    }
}

AP_CANManager& AP::can() {
    return *AP_CANManager::get_singleton();
}

#endif

