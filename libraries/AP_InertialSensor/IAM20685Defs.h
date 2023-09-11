/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2017 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

#ifndef _INV_IAM20685_DEFS_H_
#define _INV_IAM20685_DEFS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* forward declaration */
struct inv_IAM20685;

/* Device Registers */
#define IAM20685_REG_GYRO_X_DATA_OFFSET                0x00
#define IAM20685_REG_GYRO_X_DATA_BANK                  0x0
#define IAM20685_REG_GYRO_X_DATA_MASK            0xFFFF
#define IAM20685_REG_GYRO_X_DATA_POS             0x0

#define IAM20685_REG_GYRO_Y_DATA_OFFSET                0x01
#define IAM20685_REG_GYRO_Y_DATA_BANK                  0x0
#define IAM20685_REG_GYRO_Y_DATA_MASK            0xFFFF
#define IAM20685_REG_GYRO_Y_DATA_POS             0x0

#define IAM20685_REG_GYRO_Z_DATA_OFFSET                0x02
#define IAM20685_REG_GYRO_Z_DATA_BANK                  0x0
#define IAM20685_REG_GYRO_Z_DATA_MASK            0xFFFF
#define IAM20685_REG_GYRO_Z_DATA_POS             0x0

#define IAM20685_REG_TEMP1_DATA_OFFSET                 0x03
#define IAM20685_REG_TEMP1_DATA_BANK                   0x0
#define IAM20685_REG_TEMP1_DATA_MASK             0xFFFF
#define IAM20685_REG_TEMP1_DATA_POS              0x0

#define IAM20685_REG_ACCEL_X_DATA_OFFSET               0x04
#define IAM20685_REG_ACCEL_X_DATA_BANK                 0x0
#define IAM20685_REG_ACCEL_X_DATA_MASK           0xFFFF
#define IAM20685_REG_ACCEL_X_DATA_POS            0x0

#define IAM20685_REG_ACCEL_Y_DATA_OFFSET               0x05
#define IAM20685_REG_ACCEL_Y_DATA_BANK                 0x0
#define IAM20685_REG_ACCEL_Y_DATA_MASK           0xFFFF
#define IAM20685_REG_ACCEL_Y_DATA_POS            0x0

#define IAM20685_REG_ACCEL_Z_DATA_OFFSET               0x06
#define IAM20685_REG_ACCEL_Z_DATA_BANK                 0x0
#define IAM20685_REG_ACCEL_Z_DATA_MASK           0xFFFF
#define IAM20685_REG_ACCEL_Z_DATA_POS            0x0

#define IAM20685_REG_TEMP2_DATA_OFFSET                 0x07
#define IAM20685_REG_TEMP2_DATA_BANK                   0x0
#define IAM20685_REG_TEMP2_DATA_MASK             0xFFFF
#define IAM20685_REG_TEMP2_DATA_POS              0x0

#define IAM20685_REG_ACCEL_X_DATA_LR_OFFSET            0x08
#define IAM20685_REG_ACCEL_X_DATA_LR_BANK              0x0
#define IAM20685_REG_ACCEL_X_DATA_LR_MASK        0xFFFF
#define IAM20685_REG_ACCEL_X_DATA_LR_POS         0x0

#define IAM20685_REG_ACCEL_Y_DATA_LR_OFFSET            0x09
#define IAM20685_REG_ACCEL_Y_DATA_LR_BANK              0x0
#define IAM20685_REG_ACCEL_Y_DATA_LR_MASK        0xFFFF
#define IAM20685_REG_ACCEL_Y_DATA_LR_POS         0x0

#define IAM20685_REG_ACCEL_Z_DATA_LR_OFFSET            0x0A
#define IAM20685_REG_ACCEL_Z_DATA_LR_BANK              0x0
#define IAM20685_REG_ACCEL_Z_DATA_LR_MASK        0xFFFF
#define IAM20685_REG_ACCEL_Z_DATA_LR_POS         0x0

#define IAM20685_REG_FIXED_VALUE_OFFSET                0x0B
#define IAM20685_REG_FIXED_VALUE_BANK                  0x0
#define IAM20685_REG_FIXED_VALUE_MASK            0xFFFF
#define IAM20685_REG_FIXED_VALUE_POS             0x0
#define IAM20685_REG_FIXED_VALUE_DEFAULT         0xAA55

#define IAM20685_REG_FLT_Y_OFFSET                      0x0C
#define IAM20685_REG_FLT_Y_BANK                        0x0
#define IAM20685_REG_FLT_Y_MASK                  0x3F
#define IAM20685_REG_FLT_Y_POS                   0x0
#define IAM20685_REG_FLT_Y_DEFAULT               0x20

#define IAM20685_REG_FLT_Z_OFFSET                      0x0C
#define IAM20685_REG_FLT_Z_BANK                        0x0
#define IAM20685_REG_FLT_Z_MASK                  0xFC0
#define IAM20685_REG_FLT_Z_POS                   0x6
#define IAM20685_REG_FLT_Z_DEFAULT               0x20

#define IAM20685_REG_FLT_X_OFFSET                      0x0E
#define IAM20685_REG_FLT_X_BANK                        0x0
#define IAM20685_REG_FLT_X_MASK                  0x3F00
#define IAM20685_REG_FLT_X_POS                   0x8
#define IAM20685_REG_FLT_X_DEFAULT               0x20

#define IAM20685_REG_TEMP12_DELTA_OFFSET               0x0F
#define IAM20685_REG_TEMP12_DELTA_BANK                 0x0
#define IAM20685_REG_TEMP12_DELTA_MASK           0xFFFF
#define IAM20685_REG_TEMP12_DELTA_POS            0x0

#define IAM20685_REG_TEST_OFFSET                       0x17
#define IAM20685_REG_TEST_BANK                         0x0
#define IAM20685_REG_TEST_MASK                   0xFFFF
#define IAM20685_REG_TEST_POS                    0x0
#define IAM20685_REG_TEST_DEFAULT                0x0

#define IAM20685_REG_HARD_RESET_OFFSET                 0x18
#define IAM20685_REG_HARD_RESET_BANK                   0x0
#define IAM20685_REG_HARD_RESET_MASK             0x04
#define IAM20685_REG_HARD_RESET_POS              0x2
#define IAM20685_REG_HARD_RESET_DEFAULT          0x0

#define IAM20685_REG_SOFT_RESET_OFFSET                 0x18
#define IAM20685_REG_SOFT_RESET_BANK                   0x0
#define IAM20685_REG_SOFT_RESET_MASK             0x02
#define IAM20685_REG_SOFT_RESET_POS              0x1
#define IAM20685_REG_SOFT_RESET_DEFAULT          0x0

#define IAM20685_REG_REGISTER_WRITE_LOCK_OFFSET        0x19
#define IAM20685_REG_REGISTER_WRITE_LOCK_BANK          0x0
#define IAM20685_REG_REGISTER_WRITE_LOCK_MASK    0x8000
#define IAM20685_REG_REGISTER_WRITE_LOCK_POS     0xF
#define IAM20685_REG_REGISTER_WRITE_LOCK_DEFAULT 0x0

#define IAM20685_REG_CAPTURE_MODE_OFFSET               0x19
#define IAM20685_REG_CAPTURE_MODE_BANK                 0x0
#define IAM20685_REG_CAPTURE_MODE_MASK           0x08
#define IAM20685_REG_CAPTURE_MODE_POS            0x3
#define IAM20685_REG_CAPTURE_MODE_DEFAULT        0x0

#define IAM20685_REG_TCODE_STATUS_OFFSET               0x19
#define IAM20685_REG_TCODE_STATUS_BANK                 0x0
#define IAM20685_REG_TCODE_STATUS_MASK           0x07
#define IAM20685_REG_TCODE_STATUS_POS            0x0
#define IAM20685_REG_TCODE_STATUS_DEFAULT        0x7

#define IAM20685_REG_TCODE_STATUS2_MASK          0x0380
#define IAM20685_REG_TCODE_STATUS2_POS           0x7

#define IAM20685_REG_GYRO_ID_OFFSET                    0x1B
#define IAM20685_REG_GYRO_ID_BANK                      0x0
#define IAM20685_REG_GYRO_ID_MASK                0xF00
#define IAM20685_REG_GYRO_ID_POS                 0x8
#define IAM20685_REG_GYRO_ID_DEFAULT             0x0

#define IAM20685_REG_ACCEL_ID_OFFSET                   0x1B
#define IAM20685_REG_ACCEL_ID_BANK                     0x0
#define IAM20685_REG_ACCEL_ID_MASK               0x1F
#define IAM20685_REG_ACCEL_ID_POS                0x0
#define IAM20685_REG_ACCEL_ID_DEFAULT            0x0

#define IAM20685_REG_HW_REV_OFFSET                     0x1C
#define IAM20685_REG_HW_REV_BANK                       0x0
#define IAM20685_REG_HW_REV_MASK                 0x1F
#define IAM20685_REG_HW_REV_POS                  0x0
#define IAM20685_REG_HW_REV_DEFAULT              0x0

#define IAM20685_REG_ID_CODE3_OFFSET                   0x1D
#define IAM20685_REG_ID_CODE3_BANK                     0x0
#define IAM20685_REG_ID_CODE3_MASK               0xFFFF
#define IAM20685_REG_ID_CODE3_POS                0x0
#define IAM20685_REG_ID_CODE3_DEFAULT            0x0

#define IAM20685_REG_ID_CODE4_OFFSET                   0x1E
#define IAM20685_REG_ID_CODE4_BANK                     0x0
#define IAM20685_REG_ID_CODE4_MASK               0xFFFF
#define IAM20685_REG_ID_CODE4_POS                0x0
#define IAM20685_REG_ID_CODE4_DEFAULT            0x0

#define IAM20685_REG_BANK_SELECT_OFFSET                0x1F
#define IAM20685_REG_BANK_SELECT_BANK                  0x0
#define IAM20685_REG_BANK_SELECT_MASK            0xFFFF
#define IAM20685_REG_BANK_SELECT_POS             0x0
#define IAM20685_REG_BANK_SELECT_DEFAULT         0x0

#define IAM20685_REG_GYRO_DC_TRIGGER_OFFSET      0x16
#define IAM20685_REG_GYRO_DC_TRIGGER_BANK        0x0
#define IAM20685_REG_GYRO_DC_TRIGGER_MASK        0x180
#define IAM20685_REG_GYRO_DC_TRIGGER_POS         0x7
#define IAM20685_REG_GYRO_DC_TRIGGER_DEFAULT     0x0

#define IAM20685_REG_ACCEL_DC_TRIGGER_OFFSET     0x16
#define IAM20685_REG_ACCEL_DC_TRIGGER_BANK       0x0
#define IAM20685_REG_ACCEL_DC_TRIGGER_MASK       0x18
#define IAM20685_REG_ACCEL_DC_TRIGGER_POS        0x3
#define IAM20685_REG_ACCEL_DC_TRIGGER_DEFAULT    0x0

#define IAM20685_REG_MANUAL_DC_TRIGGER           {IAM20685_REG_GYRO_DC_TRIGGER_OFFSET, IAM20685_REG_GYRO_DC_TRIGGER_BANK, IAM20685_MANUAL_DC_TRIGGER_MASK}
#define IAM20685_MANUAL_DC_TRIGGER_MASK          (IAM20685_REG_GYRO_DC_TRIGGER_MASK | IAM20685_REG_ACCEL_DC_TRIGGER_MASK)

#define IAM20685_REG_WHOAMI_OFFSET                     0x0E
#define IAM20685_REG_WHOAMI_BANK                       0x1
#define IAM20685_REG_WHOAMI_MASK                 0xFF
#define IAM20685_REG_WHOAMI_POS                  0x0
#define IAM20685_REG_WHOAMI_DEFAULT              0xF3

#define IAM20685_REG_EN_ACCEL_DCST_SM_OFFSET           0x11
#define IAM20685_REG_EN_ACCEL_DCST_SM_BANK             0x1
#define IAM20685_REG_EN_ACCEL_DCST_SM_MASK      0x800
#define IAM20685_REG_EN_ACCEL_DCST_SM_POS       0xB
#define IAM20685_REG_EN_ACCEL_DCST_SM_DEFAULT   0x1

#define IAM20685_REG_EN_GYRO_DCST_SM_OFFSET            0x12
#define IAM20685_REG_EN_GYRO_DCST_SM_BANK              0x1
#define IAM20685_REG_EN_GYRO_DCST_SM_MASK       0x1000
#define IAM20685_REG_EN_GYRO_DCST_SM_POS        0xC
#define IAM20685_REG_EN_GYRO_DCST_SM_DEFAULT    0x1

#define IAM20685_REG_ODR_CONFIG_1_OFFSET                0x11
#define IAM20685_REG_ODR_CONFIG_1_BANK                  0x3
#define IAM20685_REG_ODR_CONFIG_1_MASK           0x3F00
#define IAM20685_REG_ODR_CONFIG_1_POS            0x8
#define IAM20685_REG_ODR_CONFIG_1_DEFAULT        0x0

#define IAM20685_REG_ODR_CONFIG_2_OFFSET                0x13
#define IAM20685_REG_ODR_CONFIG_2_BANK                  0x3
#define IAM20685_REG_ODR_CONFIG_2_MASK           0xF0
#define IAM20685_REG_ODR_CONFIG_2_POS            0x4
#define IAM20685_REG_ODR_CONFIG_2_DEFAULT        0x0

#define IAM20685_REG_ODR_CONFIG_3_OFFSET                0x14
#define IAM20685_REG_ODR_CONFIG_3_BANK                  0x3
#define IAM20685_REG_ODR_CONFIG_3_MASK           0x20
#define IAM20685_REG_ODR_CONFIG_3_POS            0x5
#define IAM20685_REG_ODR_CONFIG_3_DEFAULT        0x0

#define IAM20685_REG_ODR_CONFIG_4_OFFSET                0x14
#define IAM20685_REG_ODR_CONFIG_4_BANK                  0x3
#define IAM20685_REG_ODR_CONFIG_4_MASK           0x200
#define IAM20685_REG_ODR_CONFIG_4_POS            0x9
#define IAM20685_REG_ODR_CONFIG_4_DEFAULT        0x0

#define IAM20685_REG_ODR_CONFIG_5_OFFSET                0x16
#define IAM20685_REG_ODR_CONFIG_5_BANK                  0x3
#define IAM20685_REG_ODR_CONFIG_5_MASK           0x1
#define IAM20685_REG_ODR_CONFIG_5_POS            0x0
#define IAM20685_REG_ODR_CONFIG_5_DEFAULT        0x0

#define IAM20685_REG_ODR_CONFIG_6_OFFSET                0x17
#define IAM20685_REG_ODR_CONFIG_6_BANK                  0x3
#define IAM20685_REG_ODR_CONFIG_6_MASK           0x1000
#define IAM20685_REG_ODR_CONFIG_6_POS            0xC
#define IAM20685_REG_ODR_CONFIG_6_DEFAULT        0x0

#define IAM20685_REG_ACCEL_FS_SEL_OFFSET               0x14
#define IAM20685_REG_ACCEL_FS_SEL_BANK                 0x6
#define IAM20685_REG_ACCEL_FS_SEL_MASK           0x07
#define IAM20685_REG_ACCEL_FS_SEL_POS            0x0
#define IAM20685_REG_ACCEL_FS_SEL_DEFAULT        0x1

#define IAM20685_REG_GYRO_FS_SEL_OFFSET                0x14
#define IAM20685_REG_GYRO_FS_SEL_BANK                  0x7
#define IAM20685_REG_GYRO_FS_SEL_MASK            0x0F
#define IAM20685_REG_GYRO_FS_SEL_POS             0x0
#define IAM20685_REG_GYRO_FS_SEL_DEFAULT         0x5

#define IAM20685_REG_ALARMB_MASK {IAM20685_REG_ALARMB_MASK_OFFSET,IAM20685_REG_ALARMB_MASK_BANK,IAM20685_REG_ALARMB_MASK_MASK}
#define IAM20685_REG_ALARMB_MASK_OFFSET        0x17
#define IAM20685_REG_ALARMB_MASK_BANK          0x1
#define IAM20685_REG_ALARMB_MASK_MASK          0x8000
#define IAM20685_REG_ALARMB_MASK_POS           0xF
#define IAM20685_REG_ALARMB_MASK_DEFAULT       0x0

#define IAM20685_REG_SPICRC_MASK {IAM20685_REG_SPICRC_MASK_OFFSET,IAM20685_REG_SPICRC_MASK_BANK,IAM20685_REG_SPICRC_MASK_MASK}
#define IAM20685_REG_SPICRC_MASK_OFFSET        0x16
#define IAM20685_REG_SPICRC_MASK_BANK          0x1
#define IAM20685_REG_SPICRC_MASK_MASK          0x1
#define IAM20685_REG_SPICRC_MASK_POS           0x0
#define IAM20685_REG_SPICRC_MASK_DEFAULT       0x0

#define IAM20685_REG_SPICRC_LE {IAM20685_REG_SPICRC_LE_OFFSET,IAM20685_REG_SPICRC_LE_BANK,IAM20685_REG_SPICRC_LE_MASK}
#define IAM20685_REG_SPICRC_LE_OFFSET        0x19
#define IAM20685_REG_SPICRC_LE_BANK          0x1
#define IAM20685_REG_SPICRC_LE_MASK          0x0080
#define IAM20685_REG_SPICRC_LE_POS           0x7
#define IAM20685_REG_SPICRC_LE_DEFAULT       0x0

#define IAM20685_REG_SPI_CLKCNT_LE {IAM20685_REG_SPI_CLKCNT_LE_OFFSET,IAM20685_REG_SPI_CLKCNT_LE_BANK,IAM20685_REG_SPI_CLKCNT_LE_MASK}
#define IAM20685_REG_SPI_CLKCNT_LE_OFFSET        0x19
#define IAM20685_REG_SPI_CLKCNT_LE_BANK          0x1
#define IAM20685_REG_SPI_CLKCNT_LE_MASK          0x0200
#define IAM20685_REG_SPI_CLKCNT_LE_POS           0x9
#define IAM20685_REG_SPI_CLKCNT_LE_DEFAULT       0x0

#define IAM20685_REG_GYRO_ALARM {IAM20685_REG_GYRO_ALARM_OFFSET,IAM20685_REG_GYRO_ALARM_BANK,IAM20685_REG_GYRO_ALARM_MASK}
#define IAM20685_REG_GYRO_ALARM_OFFSET         0x10
#define IAM20685_REG_GYRO_ALARM_BANK           0x0
#define IAM20685_gyro_z_quadadc_alarm_MASK     0x1000
#define IAM20685_gyro_y_quadadc_alarm_MASK     0x0800
#define IAM20685_gyro_x_quadadc_alarm_MASK     0x0400
#define IAM20685_gyro_z_drfreqmeas_alarm_MASK  0x0100
#define IAM20685_gyro_y_drfreqmeas_alarm_MASK  0x0080
#define IAM20685_gyro_x_drfreqmeas_alarm_MASK  0x0040
#define IAM20685_REG_GYRO_ALARM_MASK           (IAM20685_gyro_x_quadadc_alarm_MASK|IAM20685_gyro_y_quadadc_alarm_MASK|IAM20685_gyro_z_quadadc_alarm_MASK| \
												IAM20685_gyro_x_drfreqmeas_alarm_MASK | IAM20685_gyro_y_drfreqmeas_alarm_MASK | IAM20685_gyro_z_drfreqmeas_alarm_MASK)
#define IAM20685_REG_GYRO_ALARM_DEFAULT        0x0

#define IAM20685_REG_GYRO_1_ALARM {IAM20685_REG_GYRO_1_ALARM_OFFSET,IAM20685_REG_GYRO_1_ALARM_BANK,IAM20685_REG_GYRO_1_ALARM_MASK}
#define IAM20685_REG_GYRO_1_ALARM_OFFSET         0x11
#define IAM20685_REG_GYRO_1_ALARM_BANK           0x0
#define IAM20685_gyro_z_dcst_MASK                0x4000
#define IAM20685_gyro_y_dcst_MASK                0x2000
#define IAM20685_gyro_x_dcst_MASK                0x1000
#define IAM20685_gyro_vref_ALARM_MASK            0x0010
#define IAM20685_gyro_z_drclk_alarm_MASK         0x0008
#define IAM20685_gyro_y_drclk_alarm_MASK         0x0004
#define IAM20685_gyro_x_drclk_alarm_MASK         0x0002
#define IAM20685_REG_GYRO_1_ALARM_MASK           (IAM20685_gyro_z_dcst_MASK | IAM20685_gyro_y_dcst_MASK | IAM20685_gyro_x_dcst_MASK | \
												 IAM20685_gyro_vref_ALARM_MASK | IAM20685_gyro_z_drclk_alarm_MASK | IAM20685_gyro_y_drclk_alarm_MASK | \
												 IAM20685_gyro_x_drclk_alarm_MASK)
#define IAM20685_REG_GYRO_1_ALARM_DEFAULT        0x0

#define IAM20685_REG_ACCEL_ALARM {IAM20685_REG_ACCEL_ALARM_OFFSET,IAM20685_REG_ACCEL_ALARM_BANK,IAM20685_REG_ACCEL_ALARM_MASK}
#define IAM20685_REG_ACCEL_ALARM_OFFSET          0x12
#define IAM20685_REG_ACCEL_ALARM_BANK            0x0
#define IAM20685_vrefshieldz_alarm_MASK          0x2000
#define IAM20685_vrefshieldxy_alarm_MASK         0x1000
#define IAM20685_accel_cp_ALARM_MASK             0x0002
#define IAM20685_accel_vref_ALARM_MASK           0x0001
#define IAM20685_REG_ACCEL_ALARM_MASK           (IAM20685_vrefshieldz_alarm_MASK | IAM20685_vrefshieldxy_alarm_MASK | IAM20685_accel_cp_ALARM_MASK | \
												 IAM20685_accel_vref_ALARM_MASK)
#define IAM20685_REG_ACCEL_ALARM_DEFAULT        0x0

#define IAM20685_REG_ACCEL_1_ALARM {IAM20685_REG_ACCEL_1_ALARM_OFFSET,IAM20685_REG_ACCEL_1_ALARM_BANK,IAM20685_REG_ACCEL_1_ALARM_MASK}
#define IAM20685_REG_ACCEL_1_ALARM_OFFSET        0x13
#define IAM20685_REG_ACCEL_1_ALARM_BANK          0x0
#define IAM20685_accel_x_dcst_MASK               0x0004
#define IAM20685_accel_y_dcst_MASK               0x0002
#define IAM20685_accel_z_dcst_MASK               0x0001
#define IAM20685_REG_ACCEL_1_ALARM_MASK          (IAM20685_accel_x_dcst_MASK | IAM20685_accel_y_dcst_MASK | IAM20685_accel_z_dcst_MASK)
#define IAM20685_REG_ACCEL_1_ALARM_DEFAULT        0x0

#define IAM20685_REG_OTHER_1_ALARM {IAM20685_REG_OTHER_1_ALARM_OFFSET,IAM20685_REG_OTHER_1_ALARM_BANK,IAM20685_REG_OTHER_1_ALARM_MASK}
#define IAM20685_REG_OTHER_1_ALARM_OFFSET        0x14
#define IAM20685_REG_OTHER_1_ALARM_BANK          0x0
#define IAM20685_otp_crc_alarm_MASK              0x2000
#define IAM20685_otp_reg_alarm_MASK              0x1000
#define IAM20685_bg_alarm_MASK                   0x0800
#define IAM20685_cp_vref_ALARM_MASK              0x0400
#define IAM20685_temp_vref_ALARM_MASK            0x0200
#define IAM20685_vdd_alarm_MASK                  0x0100
#define IAM20685_vddio_alarm_MASK                0x0080
#define IAM20685_dvddreg_alarm_MASK              0x0040
#define IAM20685_avddreg_alarm_MASK              0x0020
#define IAM20685_vddmaster_alarm_MASK            0x0010
#define IAM20685_temp12_alarm_MASK               0x0008
#define IAM20685_temp_dsp_alarm_MASK             0x0004
#define IAM20685_rcosc2_freqmeas_alarm_MASK      0x0002
#define IAM20685_rcosc1_freqmeas_alarm_MASK      0x0001
#define IAM20685_REG_OTHER_1_ALARM_MASK          (IAM20685_otp_crc_alarm_MASK | IAM20685_otp_reg_alarm_MASK | IAM20685_bg_alarm_MASK | \
                                                  IAM20685_cp_vref_ALARM_MASK | IAM20685_temp_vref_ALARM_MASK | IAM20685_vdd_alarm_MASK | \
												  IAM20685_vddio_alarm_MASK | IAM20685_dvddreg_alarm_MASK | IAM20685_avddreg_alarm_MASK | \
												  IAM20685_vddmaster_alarm_MASK | IAM20685_temp12_alarm_MASK | IAM20685_temp_dsp_alarm_MASK | \
												  IAM20685_rcosc2_freqmeas_alarm_MASK | IAM20685_rcosc1_freqmeas_alarm_MASK)
#define IAM20685_REG_OTHER_1_ALARM_DEFAULT        0x0

#define IAM20685_REG_OTHER_2_ALARM {IAM20685_REG_OTHER_2_ALARM_OFFSET,IAM20685_REG_OTHER_2_ALARM_BANK,IAM20685_REG_OTHER_2_ALARM_MASK}
#define IAM20685_REG_OTHER_2_ALARM_OFFSET        0x15
#define IAM20685_REG_OTHER_2_ALARM_BANK          0x0
#define IAM20685_REG_SYSCLK_ALARM_MASK           0x2000
#define IAM20685_otp_cpy_alarm_MASK              0x1000
#define IAM20685_reg_eccdone_warn_MASK           0x0200
#define IAM20685_reg_eccerr_alarm_MASK           0x0100
#define IAM20685_ahb_eccdone_warn_MASK           0x0080
#define IAM20685_ahb_eccerr_alarm_MASK           0x0040
#define IAM20685_regcheck_alarm_MASK             0x0020
#define IAM20685_REG_SPI_CLKCNT_ALARM_MASK       0x0010
#define IAM20685_spi_cmddecod_alarm_MASK         0x0008
#define IAM20685_REG_SPICRC_ALARM_MASK           0x0004
#define IAM20685_spi_ahb_bus_alarm_MASK          0x0002
#define IAM20685_REG_OTHER_2_ALARM_MASK          (IAM20685_REG_SYSCLK_ALARM_MASK | IAM20685_otp_cpy_alarm_MASK | IAM20685_reg_eccdone_warn_MASK | \
                                                  IAM20685_reg_eccerr_alarm_MASK | IAM20685_ahb_eccdone_warn_MASK | IAM20685_ahb_eccerr_alarm_MASK | \
												  IAM20685_regcheck_alarm_MASK | IAM20685_REG_SPI_CLKCNT_ALARM_MASK | IAM20685_spi_cmddecod_alarm_MASK | \
												  IAM20685_REG_SPICRC_ALARM_MASK | IAM20685_spi_ahb_bus_alarm_MASK)
#define IAM20685_REG_OTHER_2_ALARM_DEFAULT        0x0

#define IAM20685_REG_SPICRC_ALARM {IAM20685_REG_SPICRC_ALARM_OFFSET,IAM20685_REG_SPICRC_ALARM_BANK,IAM20685_REG_SPICRC_ALARM_MASK}
#define IAM20685_REG_SPICRC_ALARM_OFFSET        0x15
#define IAM20685_REG_SPICRC_ALARM_BANK          0x0
#define IAM20685_REG_SPICRC_ALARM_POS           0x2
#define IAM20685_REG_SPICRC_ALARM_DEFAULT       0x0

#define IAM20685_REG_SPI_CLKCNT_ALARM {IAM20685_REG_SPI_CLKCNT_ALARM_OFFSET,IAM20685_REG_SPI_CLKCNT_ALARM_BANK,IAM20685_REG_SPI_CLKCNT_ALARM_MASK}
#define IAM20685_REG_SPI_CLKCNT_ALARM_OFFSET        0x15
#define IAM20685_REG_SPI_CLKCNT_ALARM_BANK          0x0
#define IAM20685_REG_SPI_CLKCNT_ALARM_POS           0x4
#define IAM20685_REG_SPI_CLKCNT_ALARM_DEFAULT       0x0

#define IAM20685_REG_SM22_ALARM {IAM20685_REG_SM22_ALARM_OFFSET,IAM20685_REG_SM22_ALARM_BANK,IAM20685_REG_SM22_ALARM_MASK}
#define IAM20685_REG_SM22_ALARM_OFFSET        0x15
#define IAM20685_REG_SM22_ALARM_BANK          0x0
#define IAM20685_REG_SM22_ALARM_MASK          (IAM20685_reg_eccdone_warn_MASK | IAM20685_reg_eccerr_alarm_MASK | IAM20685_ahb_eccdone_warn_MASK | IAM20685_ahb_eccerr_alarm_MASK)
#define IAM20685_REG_SM22_ALARM_POS           (0x9 | 0x8 | 0x7 | 0x6)
#define IAM20685_REG_SM22_ALARM_DEFAULT       0x0

#define IAM20685_REG_SM30_ALARM {IAM20685_REG_SM30_ALARM_OFFSET,IAM20685_REG_SM30_ALARM_BANK,IAM20685_REG_SM30_ALARM_MASK}
#define IAM20685_REG_SM30_ALARM_OFFSET        0x15
#define IAM20685_REG_SM30_ALARM_BANK          0x0
#define IAM20685_spi_cmddecod_alarm_MASK      0x0008
#define IAM20685_REG_SM30_ALARM_MASK          (IAM20685_REG_SPI_CLKCNT_ALARM_MASK | IAM20685_REG_SPICRC_ALARM_MASK)
#define IAM20685_REG_SM30_ALARM_POS           (0x4 | 0x3 | 0x2)
#define IAM20685_REG_SM30_ALARM_DEFAULT       0x0




/* Helper macros */

#ifndef INV_MIN
#define INV_MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif

#ifndef INV_MAX
#define INV_MAX(x, y) (((x) > (y)) ? (x) : (y))
#endif

#ifndef INV_ABS
#define INV_ABS(x) (((x) < 0) ? -(x) : (x))
#endif

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _INV_IAM20685_DEFS_H_ */
