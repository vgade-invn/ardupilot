#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include "Util.h"
#include <chheap.h>

#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

using namespace ChibiOS;
/**
   how much free memory do we have in bytes.
*/
uint32_t ChibiUtil::available_memory(void)
{
    size_t totalp = 0;
    // get memory available on heap
    chHeapStatus(nullptr, &totalp, nullptr);

    // we also need to add in memory that is not yet allocated to the heap
    totalp += chCoreGetStatusX();

    return totalp;
}

/*
  get safety switch state
 */
ChibiUtil::safety_state ChibiUtil::safety_switch_state(void)
{
#if HAL_WITH_IO_MCU
    return iomcu.get_safety_switch_state();
#else
    return SAFETY_NONE;
#endif
}
    
#endif //CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
