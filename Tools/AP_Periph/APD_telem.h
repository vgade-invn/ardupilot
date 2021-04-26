/*
  ESC Telemetry for Hobbywing Pro 80A HV ESC. This will be
  incorporated into a broader ESC telemetry library in ArduPilot
  master in the future
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#ifdef HAL_PERIPH_ENABLE_APD_TELEM

class APD_Telem {
public:
    APD_Telem();

    void init(AP_HAL::UARTDriver *uart);
    bool update();

    struct APD {
        float temperature;
        float voltage;
        float current;
        float consumption;
        float eRPM;
        uint8_t status;
    };

    const APD &get_telem(void) {
        return decoded;
    }

private:
    AP_HAL::UARTDriver *uart;

    struct PACKED {
        uint8_t header; // 0xA5
        int8_t temperature;
        uint16_t bus_voltage;
        uint16_t bus_current;
        uint16_t consumption;
        uint16_t eRPM;
        uint8_t status;
        uint8_t crc;
    } pkt;

    uint8_t len;
    uint32_t last_read_ms;
    uint32_t error_count;
    uint32_t last_telem_req_ms;

    struct APD decoded;
    uint8_t last_status;

    bool parse_packet(void);
};

#endif // HAL_PERIPH_ENABLE_APD_TELEM

