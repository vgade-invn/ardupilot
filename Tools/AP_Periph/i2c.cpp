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
  AP_Periph i2c support
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "i2c.h"

#define I2C_BUSCLOCK 100000

static const I2CConfig i2cfg = {
    OPMODE_I2C,
    I2C_BUSCLOCK,
    STD_DUTY_CYCLE
};

bool i2c_transfer(uint8_t address,
                  const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len)
{
    i2cAcquireBus(&I2CD1);

    int ret;

    i2cStart(&I2CD1, &i2cfg);

    uint32_t timeout_ms = 4+2*(((8*1000000UL/I2C_BUSCLOCK)*MAX(send_len, recv_len))/1000);

    if (send_len == 0) {
        ret = i2cMasterReceiveTimeout(&I2CD1, address, recv, recv_len, chTimeMS2I(timeout_ms));
    } else {
        ret = i2cMasterTransmitTimeout(&I2CD1, address, send, send_len, recv, recv_len, chTimeMS2I(timeout_ms));
    }

    i2cStop(&I2CD1);

    i2cReleaseBus(&I2CD1);
    return ret == MSG_OK;
}
