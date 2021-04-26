/*
  ESC Telemetry for Hobbywing Pro 80A HV ESC. This will be
  incorporated into a broader ESC telemetry library in ArduPilot
  master in the future

  This protocol only allows for one ESC per UART RX line, so using a
  CAN node per ESC works well.
 */
#include "APD_telem.h"
#include <AP_Math/crc.h>
#include <AP_HAL/utility/sparse-endian.h>
#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_APD_TELEM

extern const AP_HAL::HAL& hal;

#define TELEM_HEADER 0xA5

// constructor
APD_Telem::APD_Telem(void)
{
}

void APD_Telem::init(AP_HAL::UARTDriver *_uart)
{
    uart = _uart;
    uart->set_options(AP_HAL::UARTDriver::OPTION_PULLUP_RX);
    uart->begin(115200);
}

/*
  update ESC telemetry
 */
bool APD_Telem::update()
{
    uint32_t now = AP_HAL::millis();
    if (now - last_telem_req_ms >= 50) {
        hal.rcout->set_telem_request_mask(1);
        last_telem_req_ms = now;
    }

    uint32_t n = uart->available();
    if (n == 0) {
        return false;
    }

    // we expect at least 50ms idle between frames
    bool frame_gap = (now - last_read_ms) > 10;

    last_read_ms = now;

    // don't read too much in one loop to prevent too high CPU load
    if (n > 500) {
        n = 500;
    }
    if (len == 0 && !frame_gap) {
        uart->discard_input();
        return false;
    }

    if (frame_gap) {
        len = 0;
    }

    bool ret = false;

    while (n--) {
        uint8_t b = uart->read();
        if (len == 0 && b != TELEM_HEADER) {
            continue;
        }
        uint8_t *buf = (uint8_t *)&pkt;
        buf[len++] = b;
        if (len == sizeof(pkt)) {
            ret = parse_packet();
            len = 0;
        }
    }
    return ret;
}

/*
  parse packet
 */
bool APD_Telem::parse_packet(void)
{
    uint8_t crc = crc_crc8_APD((uint8_t *)&pkt, sizeof(pkt)-1);
    if (crc != pkt.crc) {
        return false;
    }
    decoded.temperature = pkt.temperature;
    decoded.voltage = be16toh(pkt.bus_voltage) * 0.01;
    decoded.current = be16toh(pkt.bus_current) * 0.01;
    decoded.consumption = be16toh(pkt.consumption);
    decoded.eRPM = be16toh(pkt.eRPM);
    decoded.status = pkt.status;
    if (pkt.status != last_status) {
        can_printf("ESC status 0x%02x", unsigned(pkt.status));
        last_status = pkt.status;
    }
    return true;
}

#endif // HAL_PERIPH_ENABLE_APD_TELEM

