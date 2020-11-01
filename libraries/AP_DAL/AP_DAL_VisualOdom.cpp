#include "AP_DAL_VisualOdom.h"

#include <AP_VisualOdom/AP_VisualOdom.h>

#include <AP_Logger/AP_Logger.h>

AP_DAL_VisualOdom::AP_DAL_VisualOdom()
{
}

void AP_DAL_VisualOdom::start_frame(const uint64_t time_us)
{
    const auto *vo = AP::visualodom();

    RVOH.ptr_is_nullptr = (vo == nullptr);
    if (vo != nullptr) {
        RVOH.healthy = vo->healthy();
    }
    WRITE_REPLAY_BLOCK(RVOH, RVOH);
}
