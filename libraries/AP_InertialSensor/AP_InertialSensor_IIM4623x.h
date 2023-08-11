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

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_IIM4623x : public AP_InertialSensor_Backend {
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                            enum Rotation rotation);

    /**
     * Configure the sensors and start reading routine.
     */
    void start() override;
    bool update() override;

private:
    AP_InertialSensor_IIM4623x(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                enum Rotation rotation,
                                uint8_t drdy_gpio);

    /*
      initialise driver
     */
    bool init();
    void transfer_packet(uint8_t out_packet[20], int rec_pkt_len);
    void read_sensor(void);
    void loop(void);

    bool wait_dready(uint32_t timeout_us);
    bool read_register(uint8_t address, uint8_t *data, uint8_t length);
    bool write_registers(uint8_t address, const uint8_t *data, uint8_t length);
    bool write_register8(uint8_t address, uint8_t value) {
        return write_registers(address, &value, sizeof(value));
    }
    bool write_register16(uint8_t address, uint16_t value) {
        return write_registers(address, (uint8_t*)&value, sizeof(value));
    }
    bool read_streaming(uint8_t *data, uint8_t length, uint8_t &status, uint8_t &counter, uint64_t &timestamp_us);
    void start_streaming(void);

    uint16_t calc_checksum(uint8_t *buff, uint32_t length) const;

    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    uint8_t accel_instance;
    uint8_t gyro_instance;
    enum Rotation rotation;
    uint8_t drdy_pin;

    uint8_t product_id;

    float temp_sum;
    uint8_t temp_count;
    float expected_sample_rate_hz;

    uint64_t first_imu_timestamp_us;
    uint64_t first_timestamp_us;
};
