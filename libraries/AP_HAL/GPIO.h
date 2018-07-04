#pragma once

#include <stdint.h>

#include "AP_HAL_Namespace.h"

#define HAL_GPIO_INPUT  0
#define HAL_GPIO_OUTPUT 1
#define HAL_GPIO_ALT    2

class AP_HAL::DigitalSource {
public:
    virtual void    mode(uint8_t output) = 0;
    virtual uint8_t read() = 0;
    virtual void    write(uint8_t value) = 0;
    virtual void    toggle() = 0;
};

class AP_HAL::GPIO {
public:
    GPIO() {}
    virtual void    init() = 0;
    virtual void    pinMode(uint8_t pin, uint8_t output) = 0;

    // optional interface on some boards
    virtual void    pinMode(uint8_t pin, uint8_t output, uint8_t alt) {};

    virtual uint8_t read(uint8_t pin) = 0;
    virtual void    write(uint8_t pin, uint8_t value) = 0;
    virtual void    toggle(uint8_t pin) = 0;

    /* Alternative interface: */
    virtual AP_HAL::DigitalSource* channel(uint16_t n) = 0;

    enum INTERRUPT_TRIGGER_TYPE {
        INTERRUPT_NONE,
        INTERRUPT_FALLING,
        INTERRUPT_RISING,
        INTERRUPT_BOTH,
    };

    /* Interrupt interface: */
    //                                ret , pin    , state,timestamp
    FUNCTOR_TYPEDEF(irq_handler_fn_t, void, uint8_t, bool, uint32_t);
    virtual bool    attach_interrupt(uint8_t pin,
                                     irq_handler_fn_t fn,
                                     INTERRUPT_TRIGGER_TYPE mode) = 0;

    /* return true if USB cable is connected */
    virtual bool    usb_connected(void) = 0;
};
