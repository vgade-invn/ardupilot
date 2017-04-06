#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include "Semaphores.h"
#include <nuttx/arch.h>

extern const AP_HAL::HAL& hal;

using namespace PX4;

Semaphore::Semaphore(void)
{
    printf("**** SEM init %p\n", &sem);
    sem_init(&sem, 0, 0);
}

bool Semaphore::give() 
{
    printf("post %p\n", &sem);
    return sem_post(&sem) == OK;
}

bool Semaphore::take(uint32_t timeout_ms) 
{
    if (up_interrupt_context()) {
        // don't ever wait on a semaphore in interrupt context
        printf("irq wait %p %u\n", &sem, timeout_ms);
        return sem_trywait(&sem) == OK;
    }
    if (timeout_ms == 0) {
        // no timeout
        printf("nowait %p %u\n", &sem, timeout_ms);
        bool ret = (sem_wait(&sem) == OK);
        printf("nowait done %p %u %d\n", &sem, timeout_ms, ret);
        return ret;
    }
    struct timespec ts {};
    ts.tv_sec = timeout_ms / 1000;
    ts.tv_nsec = (timeout_ms % 1000) * 1000UL * 1000UL;
    printf("wait %p %u\n", &sem, timeout_ms);
    return sem_timedwait(&sem, &ts) == OK;
}

bool Semaphore::take_nonblocking() 
{
    return sem_trywait(&sem) == OK;
}

#endif // CONFIG_HAL_BOARD
