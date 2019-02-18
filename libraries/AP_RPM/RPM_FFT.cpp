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
#include <AP_AHRS/AP_AHRS.h>
#include "RPM_FFT.h"

extern const AP_HAL::HAL& hal;

/* 
   open the sensor in constructor
*/
AP_RPM_FFT::AP_RPM_FFT(AP_RPM &_ap_rpm, uint8_t _instance, AP_RPM::RPM_State &_state) :
    AP_RPM_Backend(_ap_rpm, _instance, _state)
{
    instance = _instance;
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_RPM_FFT::fast_timer_update, void));
    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_RPM_FFT::slow_timer_update, void));
}


/*
  timer function called at 1kHz
*/
void AP_RPM_FFT::fast_timer_update(void)
{
    if (nsamples < ARRAY_SIZE(fft_buffer)) {
        const AP_InertialSensor &ins = AP::ins();
        uint32_t imu_sample_us = ins.get_last_update_usec();
        if (imu_sample_us == last_imu_sample_us) {
            // no new sample from INS
            return;
        }
        last_imu_sample_us = imu_sample_us;

        const Vector3f &accel = ins.get_accel(0);
        fft_buffer[nsamples] = accel.y;
        nsamples++;
    }
}

/*
  IO timer function called at low priority to calculate FFT
*/
void AP_RPM_FFT::slow_timer_update(void)
{
    if (nsamples != ARRAY_SIZE(fft_buffer)) {
        // not ready yet
        return;
    }
    // here is where we call the fft


    WITH_SEMAPHORE(sem);
    new_rpm = 72;
    have_new_rpm = true;
}

void AP_RPM_FFT::update(void)
{
    WITH_SEMAPHORE(sem);
    if (have_new_rpm) {
        state.rate_rpm = new_rpm * ap_rpm._scaling[state.instance];
        state.signal_quality = 0.5f;
        state.last_reading_ms = AP_HAL::millis();
        have_new_rpm = false;
    }
}
