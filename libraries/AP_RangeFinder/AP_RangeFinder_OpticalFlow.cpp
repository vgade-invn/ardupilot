// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_OpticalFlow.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>

extern const AP_HAL::HAL& hal;

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_OpticalFlow::AP_RangeFinder_OpticalFlow(RangeFinder &_ranger, uint8_t instance,
                                                       RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
    ahrs = (AP_AHRS_NavEKF *)AP_Param::find_object("AHRS_");    
    flow = (OpticalFlow *)AP_Param::find_object("FLOW");
}

/* 
   detect if an optical flow sensor is available
*/
bool AP_RangeFinder_OpticalFlow::detect(RangeFinder &_ranger, uint8_t instance)
{
    AP_AHRS_NavEKF *_ahrs = (AP_AHRS_NavEKF *)AP_Param::find_object("AHRS_");    
    OpticalFlow *_flow = (OpticalFlow *)AP_Param::find_object("FLOW");
    if (_ahrs == nullptr || _flow == nullptr) {
        return false;
    }
    return _flow->enabled();
}

/* 
   update the state of the sensor
*/
void AP_RangeFinder_OpticalFlow::update(void)
{
    Vector3f velocity;
    if (!flow->enabled() || !flow->healthy() || !ahrs->get_velocity_NED(velocity)) {
        set_status(RangeFinder::RangeFinder_NoData);
        return;
    }
    if (flow->last_update() == last_flow_data_ms) {
        return;
    }
    // find ground speed in body frame X direction
    velocity = ahrs->get_rotation_body_to_ned().mul_transpose(velocity);

    // find corrected flow rate in radians/second around y axis
    float corrected_rate = flow->flowRate().y - flow->bodyRate().y;
    if (corrected_rate <= 0) {
        set_status(RangeFinder::RangeFinder_NoData);
        return;        
    }

    // the corrected rate gives the scaling factor between speed and height
    float range = velocity.x / corrected_rate;
    if (range > 0xFFFF * 0.01f) {
        // can't represent really long range in cm uint16_t
        set_status(RangeFinder::RangeFinder_OutOfRangeHigh);
        return;
    }
    state.distance_cm = range * 100;
    state.voltage_mv = degrees(corrected_rate);
    last_flow_data_ms = flow->last_update();
    update_status();
}
