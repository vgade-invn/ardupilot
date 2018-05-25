#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>

class UARTDriver : public AP_HAL::UARTDriver {
public:
    UARTDriver() {}

    /* Empty implementations of UARTDriver virtual methods */
    void begin(uint32_t b);
    void begin(uint32_t b, uint16_t rxS, uint16_t txS);
    void end();
    bool is_initialized() { return _initialised; }
    bool tx_pending();

    // unimplemented
    void flush() {}
    void set_blocking_writes(bool blocking) { (void)blocking; }

    uint32_t available() override;
    uint32_t txspace() override;
    int16_t read() override;

    size_t write(uint8_t c);
    size_t write(const uint8_t *buffer, size_t size);

    uint32_t handle_inbound(const uint8_t *buffer, uint32_t size);
    int16_t fetch_for_outbound(void);

private:
    bool _initialised;

    // ring buffers
    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
};
