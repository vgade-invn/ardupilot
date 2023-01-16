/*
  common code between HALs for UARTDriver
 */
#include "AP_HAL.h"
#include "UARTDriver.h"

void BufferModel::set_baudrate(uint32_t baudrate)
{
    // assume 33% of theoretical bandwidth
    byte_rate = baudrate / (3.0 * 10.0);
}

void BufferModel::adjust_send_size(uint32_t &send)
{
    uint32_t now = AP_HAL::micros();
    const uint32_t dt_us = now - last_update_us;
    last_update_us = now;
    avail_space += dt_us * byte_rate * 1.0e-6;
    if (avail_space < send) {
        send = avail_space;
    }
}

void BufferModel::sent(uint32_t send)
{
    avail_space -= send;
    if (avail_space < 0) {
        avail_space = 0;
    }
}
