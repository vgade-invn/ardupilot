#pragma once

#include "AP_HAL_Linux.h"
#include <AP_HAL/I2CDevice.h>

namespace Linux {

#define MR100_NUM_MOTORS 4
    
class RCOutput_MR100 : public AP_HAL::RCOutput {
public:
    static RCOutput_MR100 *from(AP_HAL::RCOutput *rcout) {
        return static_cast<RCOutput_MR100*>(rcout);
    }

    void     init() override;
    void     set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void     enable_ch(uint8_t ch) override;
    void     disable_ch(uint8_t ch) override;
    void     write(uint8_t ch, uint16_t period_us) override;
    void     cork() override;
    void     push() override;
    uint16_t read(uint8_t ch) override;
    void     read(uint16_t* period_us, uint8_t len) override;
    void     set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm) override;

private:
    int fd = -1;
    uint16_t period_us[MR100_NUM_MOTORS];
    bool     corking;
};

}
