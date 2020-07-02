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
  simulate ship takeoff/landing
*/

#include "SIM_Ship.h"
#include "SITL.h"

#include <stdio.h>

#include "SIM_Aircraft.h"
#include <AP_HAL_SITL/SITL_State.h>

using namespace SITL;

// SITL Ship parameters
const AP_Param::GroupInfo ShipSim::var_info[] = {
    AP_GROUPINFO("ENABLE",    1, ShipSim,  enable, 0),
    AP_GROUPINFO("SPEED",     2, ShipSim,  speed, 3),
    AP_GROUPINFO("PSIZE",     3, ShipSim,  path_size, 1000),
    AP_GROUPINFO("SYSID",     4, ShipSim,  sys_id, 17),
    AP_GROUPINFO("LSIZE",     5, ShipSim,  land_size, 10),
    AP_GROUPEND
};

/*
  update a simulated vehicle
 */
void Ship::update(float delta_t)
{
    float circumference = M_PI * sim->path_size.get();
    float dist = delta_t * sim->speed.get();
    float dangle = (dist / circumference) * 360.0;

    heading_deg += dangle;
    heading_deg = wrap_360(heading_deg);

    position.x += dist*cosf(radians(heading_deg));
    position.y += dist*sinf(radians(heading_deg));
}

ShipSim::ShipSim()
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  get ground speed adjustment if we are landed on the ship
 */
Vector2f ShipSim::get_ground_speed_adjustment(const Location &loc)
{
    if (!enable) {
        return Vector2f(0,0);
    }
    Location shiploc = home;
    shiploc.offset(ship.position.x, ship.position.y);
    if (loc.get_distance(shiploc) > land_size) {
        return Vector2f(0,0);
    }
    return Vector2f(cos(radians(ship.heading_deg)), sin(radians(ship.heading_deg))) * speed.get();
}

/*
  update the ADSB peripheral state
*/
void ShipSim::update(void)
{
    if (!enable) {
        return;
    }

    auto *sitl = AP::sitl();
    uint32_t now_us = AP_HAL::micros();

    if (!initialised) {
        home = sitl->state.home;
        if (home.lat == 0 && home.lng == 0) {
            return;
        }
        initialised = true;
        ::printf("ShipSim home %f %f\n", home.lat*1.0e-7, home.lng*1.0e-7);
        ship.sim = this;
        last_update_us = now_us;
        last_report_ms = AP_HAL::millis();
    }

    float dt = (now_us - last_update_us)*1.0e-6;
    last_update_us = now_us;

    ship.update(dt);

    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_report_ms >= reporting_period_ms) {
        last_report_ms = now_ms;
        send_report();
    }
}

/*
  send a report to the vehicle control code over MAVLink
*/
void ShipSim::send_report(void)
{
    if (!mavlink.connected && mav_socket.connect(target_address, target_port)) {
        ::printf("ShipSim connected to %s:%u\n", target_address, (unsigned)target_port);
        mavlink.connected = true;
    }
    if (!mavlink.connected) {
        return;
    }

    uint32_t now = AP_HAL::millis();
    mavlink_message_t msg;
    uint16_t len;
    mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
    uint8_t saved_seq = chan0_status->current_tx_seq;

    const uint8_t component_id = MAV_COMP_ID_USER10;

    if (now - last_heartbeat_ms >= 1000) {
        last_heartbeat_ms = now;
        mavlink_heartbeat_t heartbeat;
        heartbeat.type = MAV_TYPE_SURFACE_BOAT;
        heartbeat.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
        heartbeat.base_mode = 0;
        heartbeat.system_status = 0;
        heartbeat.mavlink_version = 0;
        heartbeat.custom_mode = 0;

        /*
          save and restore sequence number for chan0, as it is used by
          generated encode functions
         */
        chan0_status->current_tx_seq = mavlink.seq;
        len = mavlink_msg_heartbeat_encode(sys_id.get(),
                                           component_id,
                                           &msg, &heartbeat);
        mav_socket.send(&msg.magic, len);
    }


    /*
      send a GLOBAL_POSITION_INT messages
     */
    mavlink_global_position_int_t pos {};

    Location loc = home;
    loc.offset(ship.position.x, ship.position.y);

    pos.time_boot_ms = now;
    pos.lat = loc.lat;
    pos.lon = loc.lng;
    pos.alt = 0;
    pos.relative_alt = 0;
    pos.hdg = ship.heading_deg*100;

    chan0_status->current_tx_seq = mavlink.seq;
    len = mavlink_msg_global_position_int_encode(sys_id,
                                                 component_id,
                                                 &msg,
                                                 &pos);
    uint8_t msgbuf[len];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);
    if (len > 0) {
        mav_socket.send(msgbuf, len);
    }

    chan0_status->current_tx_seq = saved_seq;
}
