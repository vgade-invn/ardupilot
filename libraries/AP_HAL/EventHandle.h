#pragma once

#include "AP_HAL_Namespace.h"

/*
  class for waiting on an Event Object
*/
class AP_HAL::EventHandle {
public:
    virtual int32_t wait(uint64_t timeout) = 0;
};
