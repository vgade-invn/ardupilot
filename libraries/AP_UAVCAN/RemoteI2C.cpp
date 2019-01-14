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
#include <AP_HAL/I2CDevice.h>
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

/*
  perform a remote I2C transfer. This is a blocking call.
 */
bool AP_UAVCAN::remote_i2c_transfer(uint8_t busnum, uint8_t address, const uint8_t *send, uint32_t send_len,
                                    uint8_t *recv, uint32_t recv_len)
{
    debug_uavcan(0, "I2C_transfer bus=%u address=0x%02x send=%u recv=%u\n",
                 busnum, address, send_len, recv_len);
    return false;
}

/*
  handle announce of new I2C server node
 */
void AP_UAVCAN::remote_i2c_announce_callback(uint8_t node_id, const I2CAnnounceCb &cb)
{
    for (uint8_t i=0; i<ARRAY_SIZE(i2c.remote_nodes); i++) {
        if (node_id == i2c.remote_nodes[i].node_id) {
            // already have it
            break;
        }
        if (i2c.remote_nodes[i].node_id == 0) {
            if (hal.i2c_mgr->register_i2c_bus(FUNCTOR_BIND_MEMBER(&AP_UAVCAN::remote_i2c_transfer,
                                                                  bool, uint8_t, uint8_t,
                                                                  const uint8_t *, uint32_t,
                                                                  uint8_t *, uint32_t),
                                              i2c.remote_nodes[i].busnum)) {
                i2c.remote_nodes[i].node_id = node_id;
                debug_uavcan(0, "UAVCAN: registered I2C bus %u to node %u\n",
                             i2c.remote_nodes[i].busnum,
                             i2c.remote_nodes[i].node_id);
                break;
            }
        }
    }
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

