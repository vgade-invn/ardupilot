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

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN || CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_BoardConfig/AP_BoardConfig.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include <board_config.h>
#endif
#include "WheelEncoder_Quadrature.h"
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// constructor
AP_WheelEncoder_Quadrature::AP_WheelEncoder_Quadrature(AP_WheelEncoder &frontend, uint8_t instance, AP_WheelEncoder::WheelEncoder_State &state) :
	AP_WheelEncoder_Backend(frontend, instance, state)
{
    irq_handler_fn = FUNCTOR_BIND_MEMBER(&AP_WheelEncoder_Quadrature::irq_handler, void, uint8_t, bool, uint32_t);
}

// check if pin has changed and initialise gpio event callback
void AP_WheelEncoder_Quadrature::update_pin(uint8_t &pin,
                                            uint8_t new_pin,
                                            uint8_t &pin_value,
                                            AP_HAL::GPIO::irq_handler_fn_t fn)
{
    if (new_pin == pin) {
        // no change
        return;
    }

    // remove old gpio event callback if present
    hal.gpio->attach_interrupt(pin, nullptr, AP_HAL::GPIO::INTERRUPT_NONE);

    pin = new_pin;

    // install interrupt handler on rising or falling edge of gpio for pin a
    if (new_pin != 255) {
        if (!hal.gpio->attach_interrupt(pin,
                                        fn,
                                        AP_HAL::GPIO::INTERRUPT_BOTH)) {
            hal.console->printf("WEnc: Failed to attach interrupt to pin=%u\n", pin);
        } else {
            hal.console->printf("WEnc: Attached interrupt to pin=%u\n", pin);
        }
        pin_value = hal.gpio->read(pin);
    }
}

static uint32_t interrupt_count_a = 0;
static uint32_t interrupt_count_b = 0;
static uint32_t errors = 0;

void AP_WheelEncoder_Quadrature::update(void)
{
    update_pin(last_pin_a, get_pin_a(), last_pin_a_value, irq_handler_fn);
    update_pin(last_pin_b, get_pin_b(), last_pin_b_value, irq_handler_fn);

    static uint32_t last_warn_ms = 0;
    const uint32_t now = AP_HAL::millis();
    if (now - last_warn_ms > 1000) {
        last_warn_ms = now;
        gcs().send_text(MAV_SEVERITY_WARNING, "%u interrupts a=%u b=%u errors=%u", _state.instance, interrupt_count_a, interrupt_count_b, errors);
    }

    // disable interrupts to prevent race with irq_handler
    void *irqstate = hal.scheduler->disable_interrupts_save();

    // copy distance and error count so it is accessible to front end
    _state.distance_count = irq_state.distance_count;
    _state.total_count = irq_state.total_count;
    _state.error_count = irq_state.error_count;
    _state.last_reading_ms = irq_state.last_reading_ms;

    // restore interrupts
    hal.scheduler->restore_interrupts(irqstate);
}

// convert pin a and pin b state to a wheel encoder phase
uint8_t AP_WheelEncoder_Quadrature::pin_ab_to_phase(bool pin_a, bool pin_b)
{
    if (!pin_a) {
        if (!pin_b) {
            // A = 0, B = 0
            return 0;
        } else {
            // A = 0, B = 1
            return 1;
        }
    } else {
        if (!pin_b) {
            // A = 1, B = 0
            return 3;
        } else {
            // A = 1, B = 1
            return 2;
        }
    }
    return (uint8_t)pin_a << 1 | (uint8_t)pin_b;
}

void AP_WheelEncoder_Quadrature::update_phase_and_error_count(bool pin_a_now, bool pin_b_now, uint8_t &phase, int32_t &distance_count, uint32_t &total_count, uint32_t &error_count)
{
    // convert pin state before and after to phases
    uint8_t phase_after = pin_ab_to_phase(pin_a_now, pin_b_now);

    // look for invalid changes
    uint8_t step_forward = phase < 3 ? phase+1 : 0;
    uint8_t step_back = phase > 0 ? phase-1 : 3;
    if (phase_after == step_forward) {
        phase = phase_after;
        distance_count++;
    } else if (phase_after == step_back) {
        phase = phase_after;
        distance_count--;
    } else {
        error_count++;
    }
    total_count++;
}

void AP_WheelEncoder_Quadrature::irq_handler(uint8_t pin,
                                             bool pin_value,
                                             uint32_t timestamp)
{
    // sanity check
    if (last_pin_a == 0 || last_pin_b == 0) {
        return;
    }

    // update distance and error counts
    if (pin == last_pin_a) {
        last_pin_a_value = pin_value;
        interrupt_count_a++;
    } else if (pin == last_pin_b) {
        last_pin_b_value = pin_value;
        interrupt_count_b++;
    } else {
        errors++;
        return;
    };
    update_phase_and_error_count(last_pin_a_value,
                                 last_pin_b_value,
                                 irq_state.phase,
                                 irq_state.distance_count,
                                 irq_state.total_count,
                                 irq_state.error_count);

    // record update time
    irq_state.last_reading_ms = timestamp;
}

#endif // CONFIG_HAL_BOARD
