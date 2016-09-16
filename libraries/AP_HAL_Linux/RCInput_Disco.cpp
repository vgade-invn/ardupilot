/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
  this is a driver for R/C input protocols for the Disco that combines
  the SBUS decoder on /dev/uart-sbus and the 115200 decoder on
  /dev/uart-sumd
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
#include "RCInput_Disco.h"

extern const AP_HAL::HAL& hal;

using namespace Linux;

void RCInput_Disco::init()
{
    sbus.init();
    sumd.init();
}

void RCInput_Disco::_timer_tick(void)
{
    sbus._timer_tick();
    if (sbus.new_rc_input) {
        for (uint8_t i=0; i<sbus._num_channels; i++) {
            _pwm_values[i] = sbus._pwm_values[i];
            _num_channels = sbus._num_channels;
        }        
        new_rc_input = true;
    }
    sumd._timer_tick();
    if (sumd.new_rc_input) {
        for (uint8_t i=0; i<sumd._num_channels; i++) {
            _pwm_values[i] = sumd._pwm_values[i];
            _num_channels = sumd._num_channels;
        }        
        new_rc_input = true;
    }
}

#endif // CONFIG_HAL_BOARD_SUBTYPE

