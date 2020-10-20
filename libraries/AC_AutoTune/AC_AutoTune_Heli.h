/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  support for autotune of multirotors. Based on original autotune code from ArduCopter, written by Leonard Hall
  Converted to a library by Andrew Tridgell
 */

#pragma once

#define AUTOTUNE_TESTING_STEP_TIMEOUT_MS   5000U    // timeout for tuning mode's testing step
#define AUTOTUNE_RD_STEP                  0.0005f     // minimum increment when increasing/decreasing Rate D term
#define AUTOTUNE_RP_STEP                  0.005f     // minimum increment when increasing/decreasing Rate P term
#define AUTOTUNE_SP_STEP                  0.05f     // minimum increment when increasing/decreasing Stab P term
#define AUTOTUNE_PI_RATIO_FOR_TESTING      0.1f     // I is set 10x smaller than P during testing
#define AUTOTUNE_PI_RATIO_FINAL            1.0f     // I is set 1x P after testing
#define AUTOTUNE_YAW_PI_RATIO_FINAL        0.1f     // I is set 1x P after testing
#define AUTOTUNE_RD_MAX                  0.020f     // maximum Rate D value
#define AUTOTUNE_RLPF_MIN                  1.0f     // minimum Rate Yaw filter value
#define AUTOTUNE_RLPF_MAX                  20.0f     // maximum Rate Yaw filter value
#define AUTOTUNE_RP_MIN                   0.01f     // minimum Rate P value
#define AUTOTUNE_RP_MAX                    2.0f     // maximum Rate P value
#define AUTOTUNE_SP_MAX                   10.0f     // maximum Stab P value
#define AUTOTUNE_SP_MIN                    0.5f     // maximum Stab P value
#define AUTOTUNE_RP_ACCEL_MIN           4000.0f     // Minimum acceleration for Roll and Pitch
#define AUTOTUNE_Y_ACCEL_MIN            1000.0f     // Minimum acceleration for Yaw
#define AUTOTUNE_Y_FILT_FREQ              10.0f     // Autotune filter frequency when testing Yaw
#define AUTOTUNE_D_UP_DOWN_MARGIN          0.2f     // The margin below the target that we tune D in
#define AUTOTUNE_RD_BACKOFF                1.0f     // Rate D gains are reduced to 50% of their maximum value discovered during tuning
#define AUTOTUNE_RP_BACKOFF                1.0f     // Rate P gains are reduced to 97.5% of their maximum value discovered during tuning
#define AUTOTUNE_SP_BACKOFF                0.9f     // Stab P gains are reduced to 90% of their maximum value discovered during tuning
#define AUTOTUNE_ACCEL_RP_BACKOFF          1.0f     // back off from maximum acceleration
#define AUTOTUNE_ACCEL_Y_BACKOFF           1.0f     // back off from maximum acceleration
#define AUTOTUNE_FFI_RATIO_FOR_TESTING     0.5f     // I is set 2x smaller than VFF during testing
#define AUTOTUNE_FFI_RATIO_FINAL           0.5f     // I is set 0.5x VFF after testing
#define AUTOTUNE_HELI_TARGET_RATE_RLLPIT_CDS     5000   // target roll/pitch rate during AUTOTUNE FeedForward rate test

#include "AC_AutoTune.h"

class AC_AutoTune_Heli : public AC_AutoTune {
public:
    // constructor
    AC_AutoTune_Heli()
        {
//            tune_seq[0] = 4;  // RFF_UP
//            tune_seq[1] = 8;   // MAX_GAINS
//            tune_seq[2] = 0; // RD_UP
//            tune_seq[3] = 2; // RP_UP
//            tune_seq[2] = 6; // SP_UP
//            tune_seq[3] = 9; // tune complete
            tune_seq[0] = 6; // SP_UP
            tune_seq[1] = 9; // tune complete
        };
    // save gained, called on disarm
    void save_tuning_gains() override;

protected:
    void test_init() override;
    void test_run(const float dir_sign) override;
    void backup_gains_and_initialise() override;
    void do_gcs_announcements() override;
    void load_orig_gains() override;
    void load_tuned_gains() override;
    void load_intra_test_gains() override;
    void load_test_gains() override;
    void updating_rate_ff_up(float &tune_ff, float rate_target, float meas_rate, float meas_command);
    void updating_rate_p_up(float &tune_p, float *freq, float *gain, float *phase, uint8_t &frq_cnt, float gain_incr, float max_gain);
    void updating_rate_d_up(float &tune_d, float *freq, float *gain, float *phase, uint8_t &frq_cnt, max_gain_data &max_gain_d);
    void updating_rate_d_down(float &tune_d);
    void updating_angle_p_up(float &tune_p, float *freq, float *gain, float *phase, uint8_t &frq_cnt);
    void updating_angle_p_down(float &tune_p);
    void updating_max_gains(float *freq, float *gain, float *phase, uint8_t &frq_cnt, max_gain_data &max_gain_p, max_gain_data &max_gain_d, float &tune_p, float &tune_d);


    void updating_rate_p_up_all(uint8_t test_axis) override;
    void updating_rate_p_down_all(uint8_t test_axis) override;
    void updating_rate_d_up_all(uint8_t test_axis) override;
    void updating_rate_d_down_all(uint8_t test_axis) override;
    void updating_rate_ff_up_all(uint8_t test_axis) override;
    void updating_rate_ff_down_all(uint8_t test_axis) override;
    void updating_angle_p_up_all(uint8_t test_axis) override;
    void updating_angle_p_down_all(uint8_t test_axis) override;
    void updating_max_gains_all(uint8_t test_axis) override;

    void Log_AutoTune() override;
    void Log_AutoTuneDetails() override;
    void Log_Write_AutoTune(uint8_t _axis, uint8_t tune_step, float dwell_freq, float meas_gain, float meas_phase, float new_gain_rff, float new_gain_rp, float new_gain_rd, float new_gain_sp);
    void Log_Write_AutoTuneDetails(float motor_cmd, float tgt_rate_rads, float rate_rads);


private:

};
