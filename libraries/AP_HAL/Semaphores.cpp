#include "AP_HAL.h"

extern const AP_HAL::HAL &hal;

/*
  implement WithSemaphore class for WITH_SEMAPHORE() support
 */
WithSemaphore::WithSemaphore(AP_HAL::Semaphore *mtx, uint32_t line) :
    WithSemaphore(*mtx, line)
{}

WithSemaphore::WithSemaphore(AP_HAL::Semaphore &mtx, uint32_t line) :
    _mtx(mtx)
{
    _mtx.take_blocking();
}

WithSemaphore::~WithSemaphore()
{
    _mtx.give();
}
