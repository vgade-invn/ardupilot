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
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN
#include "AP_UAVCAN.h"
#include <GCS_MAVLink/GCS.h>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

#include <ardupilot/bus/I2CReqAnnounce.hpp>
#include <ardupilot/bus/I2CAnnounce.hpp>
#include <ardupilot/bus/I2C.hpp>

#define LED_DELAY_US 50000

extern const AP_HAL::HAL& hal;

#define debug_uavcan(level_debug, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(_driver_index)) { printf(fmt, ##args); }} while (0)

// message handles
static uavcan::Publisher<ardupilot::bus::I2CReqAnnounce>* i2c_req_announce[MAX_NUMBER_OF_CAN_DRIVERS];
static uavcan::ServiceClient<ardupilot::bus::I2C> *i2c_client[MAX_NUMBER_OF_CAN_DRIVERS];
static uavcan::Subscriber<ardupilot::bus::I2CAnnounce, I2CAnnounceCb> *i2c_announce[MAX_NUMBER_OF_CAN_DRIVERS];

UC_REGISTRY_BINDER(I2CAnnounceCb, ardupilot::bus::I2CAnnounce);

void AP_UAVCAN::remote_i2c_announce_callback(uint8_t node_id, const I2CAnnounceCb &cb)
{
    printf("Got I2CAnnounce from %u\n", node_id);
}

void AP_UAVCAN::remote_i2c_announce_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const I2CAnnounceCb &cb)
{
    ap_uavcan->remote_i2c_announce_callback(node_id, cb);
}

bool AP_UAVCAN::remote_i2c_init(uint8_t driver_index)
{
    // setup I2C broadcasts and client
    i2c_req_announce[driver_index] = new uavcan::Publisher<ardupilot::bus::I2CReqAnnounce>(*_node);

    // create i2c service client
    i2c_client[driver_index] = new uavcan::ServiceClient<ardupilot::bus::I2C>(*_node);
    if (!i2c_client[driver_index] || i2c_client[driver_index]->init() < 0) {
        debug_uavcan(1, "UAVCAN: failed to init I2C service\n");
        return false;
    }
    i2c_client[driver_index]->setCallback([](const uavcan::ServiceCallResult<ardupilot::bus::I2C>&call_result) {
            printf("I2C: %u\n", call_result.isSuccessful());
        });

    // subscribe to I2CAnnounce messages
    i2c_announce[driver_index] = new uavcan::Subscriber<ardupilot::bus::I2CAnnounce, I2CAnnounceCb>(*_node);
    i2c_announce[driver_index]->start(I2CAnnounceCb(this, &remote_i2c_announce_trampoline));

    return true;
}

/*
  discover and handle remote I2C devices
 */
void AP_UAVCAN::remote_i2c_update(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - i2c.last_discover_ms >= 1000) {
        i2c.last_discover_ms = now;
        ardupilot::bus::I2CReqAnnounce msg;
        i2c_req_announce[_driver_index]->broadcast(msg);
    }
}

#endif // HAL_WITH_UAVCAN

