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
    
    uart = hal.uartC;
    uart->begin(115200);

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
            // telemetry request is encoded in the top bit
            bool telem_request = false;
            if (v & 0x8000) {
                telem_request = true;
                v &= 0x7FFF;
            }
            uint16_t pwm = v?(1000 + v):0;
            SRV_Channel::Aux_servo_function_t motor = SRV_Channels::get_motor_function(i);
            uint8_t motor_chan;
            if (telem_request && SRV_Channels::find_channel(motor, motor_chan)) {
                send_telem_packet(uint8_t(motor - SRV_Channel::k_motor1));
            }
            SRV_Channels::set_output_pwm(motor, pwm);
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

/*
  send a telemetry packet back to the master
 */
void DShotExpander::send_telem_packet(uint8_t esc_num)
{
    AP_BLHeli *blh = AP_BLHeli::get_instance();
    if (!blh) {
        return;
    }
    AP_BLHeli::telem_data td;
    uint32_t telem_ms;
    if (!blh->get_telem_data(esc_num, td, telem_ms) ||
        AP_HAL::millis() - telem_ms > 1000) {
        return;
    }
    uint8_t buf[10];
    buf[0] = td.temperature;
    buf[1] = td.voltage >> 8;
    buf[2] = td.voltage & 0xFF;
    buf[3] = td.current >> 8;
    buf[4] = td.current & 0xFF;
    buf[5] = td.consumption >> 8;
    buf[6] = td.consumption & 0xFF;
    buf[7] = td.rpm >> 8;
    buf[8] = td.rpm & 0xFF;

    uint8_t crc = 0;
    for (uint8_t i=0; i<9; i++) {    
        crc = AP_BLHeli::telem_crc8(buf[i], crc);
    }
    buf[9] = crc;
    if (uart->txspace() >= 10) {
        uart->write(buf, 10);
    }
}
