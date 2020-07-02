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
  ship landing code for quadplanes
 */

#include "Plane.h"


/*
  is ship landing enabled
*/
bool QuadPlane::ship_landing_enabled(void) const
{
    return available() && ((options & OPTION_SHIP_LANDING) != 0);
}

/*
  init for ship landing in RTL
*/
void QuadPlane::ship_landing_RTL_init(void)
{
    ship_landing.stage = ship_landing.HOLDOFF;
    ship_landing.reached_alt = false;
    ship_landing_RTL_update();
}


/*
  update for ship landing in RTL
*/
void QuadPlane::ship_landing_RTL_update(void)
{
    uint32_t now = AP_HAL::millis();
    if (ship_landing.last_update_ms == plane.land_target.last_update_ms ||
        now - plane.land_target.last_update_ms > 30*1000U) {
        // don't update position if older than 30s or not changed
        return;
    }
    ship_landing.last_update_ms = plane.land_target.last_update_ms;

    // extrapolate target position
    float dt = (now - plane.land_target.last_update_ms) * 0.001;
    Location loc = plane.land_target.loc;
    const Vector3f &vel = plane.land_target.velocity;
    loc.offset(vel.x*dt, vel.y*dt);

    IGNORE_RETURN(plane.ahrs.set_home(loc));

    if (ship_landing.stage == ship_landing.HOLDOFF) {
        // hold loiter behind and to one side
        const float radius = plane.aparm.loiter_radius;
        const float holdoff_dist = radius*1.5;
        Vector2f ofs(-fabsf(holdoff_dist), holdoff_dist);
        ofs.rotate(radians(plane.land_target.heading_cd*0.01));
        loc.offset(ofs.x, ofs.y);
        loc.alt += plane.g.RTL_altitude_cm;

        // we consider the alt to be reached if we get below 5m above
        // the target
        if (plane.current_loc.alt < loc.alt+500) {
            ship_landing.reached_alt = true;
        }

        int16_t throttle_in = plane.channel_throttle->get_control_in();
        if (throttle_in <= 0) {
            // go to approach stage when throttle is low, we are
            // pointing at the ship and have reached target alt
            float target_bearing_deg = wrap_180(degrees(plane.current_loc.get_bearing(plane.land_target.loc)));
            float ground_bearing_deg = wrap_180(degrees(plane.ahrs.groundspeed_vector().angle()));
            const float margin = 10;
            if (ship_landing.reached_alt &&
                fabsf(wrap_180(target_bearing_deg - ground_bearing_deg)) < margin) {
                ship_landing.stage = ship_landing.APPROACH;
            }
        }
    } else if (ship_landing.stage == ship_landing.APPROACH) {
        // fly directly towards the ship, at QRTL_ALT
        loc.alt += qrtl_alt;
    }

    plane.next_WP_loc = loc;
}
