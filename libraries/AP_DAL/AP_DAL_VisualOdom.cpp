#include "AP_DAL_VisualOdom.h"

#include <AP_VisualOdom/AP_VisualOdom.h>

#include <AP_Logger/AP_Logger.h>

AP_DAL_VisualOdom::AP_DAL_VisualOdom()
{
    RVOH.head1 = HEAD_BYTE1;
    RVOH.head2 = HEAD_BYTE2;
    RVOH.msgid = LOG_RVOH_MSG;
}

void AP_DAL_VisualOdom::start_frame(const uint64_t time_us)
{
    const auto *vo = AP::visualodom();

    auto &logger = AP::logger();

    RVOH.time_us = time_us;
    RVOH.ptr_is_nullptr = (vo == nullptr);
    if (vo != nullptr) {
        RVOH.healthy = vo->healthy();
    }
    logger.WriteReplayBlock(&RVOH, sizeof(RVOH));
}
