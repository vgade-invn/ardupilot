/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor_Backend.h"
enum inv_reg_rw { INV_REG_W, INV_REG_R, INV_REG_RW_MAX };
typedef struct inv_iam20685 {
	
	uint32_t                  dummy_cmd; /* a harmless command used to read back response */
} inv_iam20685_t;


class AP_InertialSensor_IAM20685: public AP_InertialSensor_Backend {
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                            enum Rotation rotation,
                                            int8_t drdy_pin);

    /**
     * Configure the sensors and start reading routine.
     */
    void start() override;
    bool update() override;

private:
    AP_InertialSensor_IAM20685(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                enum Rotation rotation,
                                int8_t drdy_gpio);

    /*
      initialise driver
     */
    bool init();
    void transfer_packet(uint8_t out_packet[4], uint8_t rec_packet[4]);
    uint32_t build_cmd(uint8_t offset, uint16_t data, enum inv_reg_rw rw);
    void inv_iam20685_read_reg(struct inv_iam20685 *s, uint8_t offset, uint16_t *data);
    void inv_iam20685_write_reg(struct inv_iam20685 *s, uint8_t offset, const uint16_t *data);
    void send_cmd(struct inv_iam20685 *s, uint32_t *cmd, uint32_t *rsp, int len);
    void read_sensor(void);
    void loop(void);
    void setup_iam(void);
    void GetSensorData(void);
    void inv_iam20685_release_capture_mode(struct inv_iam20685 *s);
    void inv_iam20685_set_hard_reset(struct inv_iam20685 *s);
    void inv_iam20685_get_fixed_value(struct inv_iam20685 *s, uint16_t *fixed_value);
    void inv_iam20685_unlock_chip(struct inv_iam20685 *s);
    void inv_iam20685_select_bank(struct inv_iam20685 *s, uint16_t bank);
    void inv_iam20685_get_whoami(struct inv_iam20685 *s, uint16_t *whoami);
    void inv_iam20685_set_capture_mode(struct inv_iam20685 *s);
    bool wait_dready(uint32_t timeout_us);
    void send_command(uint8_t command_type, uint8_t *data, uint8_t length);
    bool read_reply(uint8_t *data, uint8_t length);
    bool read_register(uint8_t address, uint8_t *data, uint8_t length);
    bool write_registers(uint8_t address, const uint8_t *data, uint8_t length);
    bool write_register8(uint8_t address, uint8_t value);
    bool write_register16(uint8_t address, uint16_t value);
    bool read_version(uint8_t &major, uint8_t &minor);
    bool read_serial(uint8_t serial[16]);
    bool read_streaming(uint8_t *data, uint8_t length, uint8_t &status, uint8_t &counter, uint64_t &timestamp_us);
    void start_streaming(void);

    uint16_t calc_checksum(uint8_t *buff, uint32_t length) const;

    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    uint8_t accel_instance;
    uint8_t gyro_instance;
    enum Rotation rotation;
    int8_t drdy_pin;
    uint8_t dev_type;
    const char *dev_name;

    uint8_t product_id;

    float temp_sum;
    uint8_t temp_count;
    float expected_sample_rate_hz;

    uint64_t first_imu_timestamp_us;
    uint64_t first_timestamp_us;
};
