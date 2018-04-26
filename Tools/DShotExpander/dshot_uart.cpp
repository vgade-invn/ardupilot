#include "DShotExpander.h"

#include <AP_Math/crc.h>

#include "ch.h"

void DShotExpander::setup_uart(void)
{
    chThdCreateFromHeap(NULL,
                        THD_WORKING_AREA_SIZE(1024),
                        "DShotUart",
                        192,
                        uart_start,
                        this);
}

#define APM_DSHOT_MAGIC 0x82

/*
  trampoline for starting thread
 */
void DShotExpander::uart_start(void *ctx)
{
    ((DShotExpander *)ctx)->uart_thread();
}

/*
  main thread for processing uart protocol commands
 */
void DShotExpander::uart_thread(void)
{
    const uint8_t max_len = 4 + max_channels*2;
    uint8_t pkt[4+max_channels*2];
    
    uart = hal.uartB;
    uart->begin(921600);

    while (true) {
        // wait for min number of bytes
        if (!uart->wait_timeout(4, 20)) {
            if (AP_HAL::millis() - last_packet_ms > 100) {
                stop_motors();
            }
            continue;
        }
        int16_t b;
        uint8_t nread = 0;
        b = uart->read();
        if (b < 0) {
            pkt_errors++;
            continue;
        }
        pkt[0] = b;
        b = uart->read();
        if (b < 0) {
            pkt_errors++;
            continue;
        }
        pkt[1] = b;
        if (pkt[0] != APM_DSHOT_MAGIC || pkt[1] > max_len) {
            // bad packet, discard
            hal.scheduler->delay_microseconds(300);
            while (uart->available() > 0) {
                uart->read();
            }
            pkt_errors++;
            continue;
        }
        if (!uart->wait_timeout(pkt[1]-2, 1)) {
            pkt_errors++;
            continue;
        }
        for (uint8_t i=2; i<pkt[1]; i++) {
            b = uart->read();
            if (b < 0) {
                break;
            }
            pkt[i] = b;
            nread = i+1;
        }
        if (nread != pkt[1]) {
            pkt_errors++;
            continue;
        }
        uint16_t crc = crc_xmodem(pkt, nread-2);
        uint16_t crc2 = pkt[nread-2] | (pkt[nread-1]<<8);
        if (crc != crc2) {
            hal.console->printf("err %u nread=%u 0x%02x 0x%02x\n", __LINE__, nread, crc, crc2);
            crc_errors++;
            continue;
        }

        uint8_t nchan = (nread - 4)/2;
        for (uint8_t i=0; i<nchan; i++) {
            uint16_t v = pkt[2+i*2] | (pkt[3+i*2]<<8);
            uint16_t pwm = 1000 + v / 2;
            SRV_Channels::set_output_pwm(SRV_Channels::get_motor_function(i), pwm);
        }

        last_packet_ms = AP_HAL::millis();
        SRV_Channels::calc_pwm();
        SRV_Channels::cork();
        SRV_Channels::output_ch_all();
        SRV_Channels::push();    
    }
}

void DShotExpander::stop_motors()
{
    for (uint8_t i=0; i<max_channels; i++) {
        SRV_Channels::set_output_pwm(SRV_Channels::get_motor_function(i), 0);
        SRV_Channels::calc_pwm();
        SRV_Channels::cork();
        SRV_Channels::output_ch_all();
        SRV_Channels::push();    
    }
}
