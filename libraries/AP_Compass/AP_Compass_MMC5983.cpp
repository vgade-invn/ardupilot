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
/*
  Driver by Andrew Tridgell, Nov 2019
 */
#include "AP_Compass_MMC5983.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <stdio.h>

extern const AP_HAL::HAL &hal;

#define REG_PRODUCT_ID      0x2F
#define REG_XOUT_L          0x00
#define REG_TOUT            0x07
#define REG_STATUS          0x08
#define REG_CONTROL0        0x09
#define REG_CONTROL1        0x0A
#define REG_CONTROL2        0x0B
#define REG_CONTROL3        0x0C

// bits in REG_CONTROL0
#define REG_CONTROL0_RESET   0x10
#define REG_CONTROL0_SET     0x08
#define REG_CONTROL0_TM_T    0x02
#define REG_CONTROL0_TM_M    0x01
#define REG_CONTROL0_AUTO_SR 0x20

// bits in REG_CONTROL1
#define REG_CONTROL1_SW_RST  0x80
#define REG_CONTROL1_BW1     0x02
#define REG_CONTROL1_BW0     0x01

// bits in REG_CONTROL2
#define REG_CONTROL2_PRD_SET_EN   0x80
#define REG_CONTROL2_PRD_SET_1    0x00
#define REG_CONTROL2_PRD_SET_25   0x10
#define REG_CONTROL2_PRD_SET_75   0x20
#define REG_CONTROL2_PRD_SET_100  0x30
#define REG_CONTROL2_PRD_SET_250  0x40
#define REG_CONTROL2_PRD_SET_500  0x50
#define REG_CONTROL2_PRD_SET_1000 0x60
#define REG_CONTROL2_PRD_SET_2000 0x70
#define REG_CONTROL2_CMM_EN       0x08
#define REG_CONTROL2_CMM_1HZ      0x01
#define REG_CONTROL2_CMM_10HZ     0x02
#define REG_CONTROL2_CMM_20HZ     0x03
#define REG_CONTROL2_CMM_50HZ     0x04
#define REG_CONTROL2_CMM_100HZ    0x05


AP_Compass_Backend *AP_Compass_MMC5983::probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                              bool force_external,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_Compass_MMC5983 *sensor = new AP_Compass_MMC5983(std::move(dev), force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_MMC5983::AP_Compass_MMC5983(AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                       bool _force_external,
                                       enum Rotation _rotation)
    : dev(std::move(_dev))
    , force_external(_force_external)
    , rotation(_rotation)
{
}

bool AP_Compass_MMC5983::init()
{
    if (!dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    dev->set_retries(10);
    
    uint8_t whoami;
    if (!dev->read_registers(REG_PRODUCT_ID, &whoami, 1) ||
        whoami != 0x30) {
        // not a MMC5983
        dev->get_semaphore()->give();
        return false;
    }

    // reset sensor
    dev->write_register(REG_CONTROL1, REG_CONTROL1_SW_RST);
    hal.scheduler->delay(10);

    // setup for 50Hz continuous, with PRD every 500 samples (every 10s)
    dev->write_register(REG_CONTROL2, REG_CONTROL2_PRD_SET_EN | REG_CONTROL2_PRD_SET_500 | REG_CONTROL2_CMM_EN | REG_CONTROL2_CMM_50HZ);

    dev->get_semaphore()->give();

    /* register the compass instance in the frontend */
    compass_instance = register_compass();

    printf("Found a MMC5983 on 0x%x as compass %u\n", dev->get_bus_id(), compass_instance);
    
    set_rotation(compass_instance, rotation);

    if (force_external) {
        set_external(compass_instance, true);
    }
    
    dev->set_device_type(DEVTYPE_MMC5983);
    set_dev_id(compass_instance, dev->get_bus_id());

    dev->set_retries(1);
    
    // call timer() at 50Hz
    dev->register_periodic_callback(20000,
                                    FUNCTOR_BIND_MEMBER(&AP_Compass_MMC5983::timer, void));

    // wait 250ms for the compass to make it's initial readings
    hal.scheduler->delay(250);
    
    return true;
}

void AP_Compass_MMC5983::timer()
{
    const uint16_t zero_offset = 32768; // 16 bit mode
    const uint16_t sensitivity = 4096; // counts per Gauss, 16 bit mode
    const float counts_to_milliGauss = 1.0e3f / sensitivity;

    uint32_t now = AP_HAL::millis();
    if (now - last_sample_ms > 500) {
        // setup for 50Hz continuous, with PRD every 500 samples (every 10s)
        dev->write_register(REG_CONTROL2, REG_CONTROL2_PRD_SET_EN | REG_CONTROL2_PRD_SET_500 | REG_CONTROL2_CMM_EN | REG_CONTROL2_CMM_50HZ);
    }
    
    uint16_t data[3];
    if (!dev->read_registers(REG_XOUT_L, (uint8_t *)&data[0], sizeof(data))) {
        return;
    }
    /*
      calculate field and offset
    */
    Vector3f field(float(be16toh(data[0])) - zero_offset,
                   float(be16toh(data[1])) - zero_offset,
                   float(be16toh(data[2])) - zero_offset);
    field *= counts_to_milliGauss;

    last_sample_ms = AP_HAL::millis();

    accumulate_sample(field, compass_instance);
}

void AP_Compass_MMC5983::read()
{
    drain_accumulated_samples(compass_instance);
}
