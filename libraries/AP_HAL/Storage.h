#pragma once

#include <stdint.h>
#include "AP_HAL_Namespace.h"

class AP_HAL::Storage {
public:
    virtual void init() = 0;
    virtual void read_block(void *dst, uint16_t src, size_t n) = 0;
    virtual void write_block(uint16_t dst, const void* src, size_t n) = 0;

    // block until all data is on stable storage. Used by FactoryTest
    virtual bool sync(void) { return false; }

    // discard all cached data and re-open storage. Used by FactoryTest
    virtual bool reopen(void) { return false; }
};
