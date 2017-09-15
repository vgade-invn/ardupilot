#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MR100
#include "RCOutput_MR100.h"

#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <utility>

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>

#include "Util.h"

using namespace Linux;

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void RCOutput_MR100::init()
{
    fd = open("/dev/STM8s", O_WRONLY);
    if (fd == -1) {
        printf("Failed to open STM8s RCOUT\n");
    }
}

void RCOutput_MR100::set_freq(uint32_t chmask, uint16_t freq_hz)
{
}

uint16_t RCOutput_MR100::get_freq(uint8_t ch)
{
    return 400;
}

void RCOutput_MR100::enable_ch(uint8_t ch)
{
}

void RCOutput_MR100::disable_ch(uint8_t ch)
{
}

void RCOutput_MR100::write(uint8_t ch, uint16_t _period_us)
{
    if (ch >= MR100_NUM_MOTORS) {
        return;
    }

    period_us[ch] = _period_us;

    if (!corking) {
        push();
    }
}

void RCOutput_MR100::cork()
{
    corking = true;
}

void RCOutput_MR100::push()
{
    if (!corking || fd == -1) {
        return;
    }
    corking = false;

    uint8_t b[5] {};
    // byte order: FL, RL, RR, FR
    const uint8_t motor_map[4] = { 2, 1, 3, 0 };
    
    uint16_t v[4];
    for (uint8_t i=0; i<MR100_NUM_MOTORS; i++) {
        v[i] = period_us[motor_map[i]];
    }
    bool all_zero = true;
    for (uint8_t i=0; i<MR100_NUM_MOTORS; i++) {
        uint16_t m;
        if (v[i] < 1000) {
            m = 0;
        } else if (v[i] > 2000) {
            m = 1000;
        } else {
            m = v[i] - 1000;
        }
        if (m != 0) {
            b[i] = m & 0xFF;
            b[4] |= (m>>8) << (i*2);
            all_zero = false;
        }
    }
    if (!all_zero) {
        printf("%u %u %u %u 0x%02x\n", b[0], b[1], b[2], b[3], b[4]);
    }
    ::write(fd, b, 5);
}

uint16_t RCOutput_MR100::read(uint8_t ch)
{
    if (ch < MR100_NUM_MOTORS) {
        return period_us[ch];
    }
    return 0;
}

void RCOutput_MR100::read(uint16_t* _period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        _period_us[i] = read(0 + i);
    }
}

void RCOutput_MR100::set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm)
{
}

#endif
