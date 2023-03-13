#pragma once

#if AP_DDS_ENABLED

#include "generated/Time.h"

void update_topic(builtin_interfaces_msg_Time* msg);

#endif // AP_DDS_ENABLED
