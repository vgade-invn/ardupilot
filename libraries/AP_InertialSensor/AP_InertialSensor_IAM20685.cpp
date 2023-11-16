#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>
#include <cmath>
#include "AP_InertialSensor_IAM20685.h"
#include "IAM20685Defs.h"
/*
  device registers
 */


#define IAM20685_WHO_AM_I_ADDR 0x75
#define REG_BANK_SEL_ADDR 0x76
#define WHO_AM_I_BANK 0x00
#define EXPECTED_WHOAMI 0xF2

//ACCEL_FS_SEL[2:0]  FS         FS_LR
//000                16.384g    32.768g
//001                16.384g    65.536g
//010                32.768g    32.768g
//011                32.768g    65.536g
//100                 2.048g     4.096g
//101                 2.048g    16.384g
//110                 4.096g     4.096g
//111                 4.096g     8.192g
#define ACCEL_FS_SEL 3

//GYRO_FS_SEL[3:0] FS
//0000             +/-328dps
//0001             +/-655 dps
//0010             +/-1311 dps
//0011             +/-1966 dps
//0100             +/-218 dps
//0101             +/-437 dps
//0110             +/-874 dps
//0111             +/-1311 dps
//1000             +/-61 dps
//1001             +/-123 dps
//1010             +/-246 dps
//1011             +/-492 dps
//1100             +/-41 dps
//1101             +/-82 dps
//1110             +/-164 dps
//1111             +/-328 dps
#define GYRO_FS_SEL 3

// Value range: 1 ~ 42. Please refer to IAM20685 datasheet v1.0 Table 15 for details.
#define X_AXIS_FILTER 32

// Value range: 1 ~ 42. Please refer to IAM20685 datasheet v1.0 Table 14 for details.
#define Y_AXIS_FILTER 32

// Value range: 1 ~ 42. Please refer to IAM20685 datasheet v1.0 Table 13 for details.
#define Z_AXIS_FILTER 32

/**
 * enum inv_reg_rw - Write or Read of register access.
 * @INV_REG_W: Write.
 * @INV_REG_R: Read.
 */

/*
  timings
 */
#define T_STALL_US   20U
#define T_RESET_MS   500U

#define TIMING_DEBUG 0
#if TIMING_DEBUG
#define DEBUG_SET_PIN(n,v) hal.gpio->write(52+n, v)
#define DEBUG_TOGGLE_PIN(n) hal.gpio->toggle(52+n)
#else
#define DEBUG_SET_PIN(n,v)
#define DEBUG_TOGGLE_PIN(n)
#endif
typedef struct inv_dev_config {
	//uint32_t odr_us;         // Output Data Rate in micro second
	uint8_t accel_fs_sel; // Accel Full Scale selection
	uint8_t gyro_fs_sel; // Gyro Full Scale selection
	uint8_t x_axis_filter; // X axis filter selection
	uint8_t y_axis_filter; // Y axis filter selection
	uint8_t z_axis_filter; // Z axis filter selection
} inv_dev_config_t;


extern const AP_HAL::HAL& hal;
void setup_iam(void);
static inv_iam20685_t icm_device;

uint32_t dummy_cmd = 0xAD00007C;
AP_InertialSensor_IIM4623x::AP_InertialSensor_IIM4623x(AP_InertialSensor &imu,
                                                         AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                                         enum Rotation _rotation)
                                                        //  int8_t _drdy_pin)
    : AP_InertialSensor_Backend(imu)
    , dev(std::move(_dev))
    , rotation(_rotation)
    // , drdy_pin(_drdy_pin)
{
}

AP_InertialSensor_Backend *AP_InertialSensor_IIM4623x::probe(AP_InertialSensor &imu,
                                                             AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                             enum Rotation rotation)
                                                            //  int8_t drdy_pin)
{
    if (!dev) {
        return nullptr;
    }
    AP_InertialSensor_IIM4623x *sensor = new AP_InertialSensor_IIM4623x(imu, std::move(dev), rotation);

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_IIM4623x::start()
{
    if (!_imu.register_accel(accel_instance, expected_sample_rate_hz, dev->get_bus_id_devtype(DEVTYPE_INS_IIM46234)) ||
        !_imu.register_gyro(gyro_instance, expected_sample_rate_hz,   dev->get_bus_id_devtype(DEVTYPE_INS_IIM46234))) {
        return;
    }

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    /*
      as the sensor does not have a FIFO we need to jump through some
      hoops to ensure we don't lose any samples. This creates a thread
      to do the capture, running at very high priority
     */
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_IIM4623x::loop, void),
                                      "IAM20685",
                                      1024, AP_HAL::Scheduler::PRIORITY_BOOST, 1)) {
        AP_HAL::panic("Failed to create IAM20685 thread");
    }
}




#define IIM_CMD_RD_BIT 0
#define IIM_CMD_WR_BIT 1
static uint8_t generate_crc(uint32_t in)
{
	int     i;
	uint8_t crc     = 0xff;
	uint8_t crc_new = 0x00;

	/* Generate CRC following datasheet */
	for (i = 23; i >= 0; i--) {
		/* CRC_New[7] = CRC[6] */
		crc_new = (uint8_t)0x80 & (crc << 1);
		/* CRC_New[6] = CRC[5] */
		crc_new |= (uint8_t)0x40 & (crc << 1);
		/* CRC_New[5] = CRC[4] */
		crc_new |= (uint8_t)0x20 & (crc << 1);
		/* CRC_New[4] = CRC[3] ^ CRC[7] */
		crc_new |= (uint8_t)0x10 & ((crc << 1) ^ (crc >> 3));
		/* CRC_New[3] = CRC[2] ^ CRC[7] */
		crc_new |= (uint8_t)0x08 & ((crc << 1) ^ (crc >> 4));
		/* CRC_New[2] = CRC[1] ^ CRC[7] */
		crc_new |= (uint8_t)0x04 & ((crc << 1) ^ (crc >> 5));
		/* CRC_New[1] = CRC[0] */
		crc_new |= (uint8_t)0x02 & (crc << 1);
		/* CRC_New[0] = Input_data[i] ^ CRC[7] */
		crc_new |= (uint8_t)0x01 & ((uint8_t)(in >> (uint32_t)i) ^ (crc >> 7));
		crc = crc_new;
	}
	crc = crc ^ (uint8_t)0xff;
	return crc;
}
uint32_t AP_InertialSensor_IIM4623x::build_cmd(uint8_t offset, uint16_t data, enum inv_reg_rw rw)
{
	uint32_t cmd;
	uint8_t  rw_bit;
	uint16_t d = data;

	if (rw == INV_REG_W) {
		rw_bit = IIM_CMD_WR_BIT;
	} else {
		rw_bit = IIM_CMD_RD_BIT;
	}

	/* data is 0 for read command */
	if (rw != INV_REG_W) {
		d = 0;
	}

	cmd = ((uint32_t)rw_bit << 31) & (uint32_t)0x80000000;
	cmd |= ((uint32_t)offset << 26) & (uint32_t)0x7c000000;
	cmd |= ((uint32_t)d << 8) & (uint32_t)0x00ffff00;
	cmd |= generate_crc(cmd >> 8) & (uint32_t)0x000000ff;
	//INV_MSG(INV_MSG_LEVEL_INFO, "generated cmd: 0x%08x\n", cmd);
	cmd = ( cmd >> 24 ) | (( cmd << 8) & 0x00ff0000 )| ((cmd >> 8) & 0x0000ff00) | ( cmd << 24)  ;
	return cmd;
}


void AP_InertialSensor_IIM4623x::transfer_packet(uint8_t out_packet[4], uint8_t rec_packet[4]){
    WITH_SEMAPHORE(dev->get_semaphore());
    
    // dev->transfer(out_packet,4,(uint8_t *)&rec_packet, 4);    
    dev->transfer(out_packet,4,nullptr, 0);
    // count =0;
    // while (!hal.gpio->read(drdy_pin)){
    //     count = count+1;
    //     if (count>2000){
    //         break;
    //     }
        
    // }
    // count =0;
    //hal.scheduler->delay(1);
    dev->transfer(nullptr, 0, rec_packet, 4);
    //hal.scheduler->delay(1);
}

void AP_InertialSensor_IIM4623x::send_cmd(struct inv_iam20685 *s, uint32_t *cmd, uint32_t *rsp, int len)
{
	
	
	//*cmd = ((*cmd & 0xFF) << 24) | ((*cmd & 0xFF00) << 8) | ((*cmd & 0xFF0000) >> 8) | ((*cmd & 0xFF000000) >> 24);

	uint8_t                    out[4]; // spi outbuf, big endian
	uint8_t                    in[4]; // spi inbuf, big endian
	uint32_t                   cmd_to_out;
	int                        i, t, shift;

	for (i = 0; i < len; i++) {
		cmd_to_out = cmd[i];
		cmd_to_out = ((cmd_to_out & 0xFF) << 24) | ((cmd_to_out & 0xFF00) << 8) | ((cmd_to_out & 0xFF0000) >> 8) | ((cmd_to_out & 0xFF000000) >> 24);
		for (t = 0; t < 4; t++) {
			shift  = 24 - (t * 8);
			out[t] = cmd_to_out >> (uint32_t)shift;

		}
		transfer_packet(out, in);
		//serif->transaction(serif->context, out, in, 4);
		
		rsp[i] = 0;
		for (t = 0; t < 4; t++) {
			shift = 24 - (t * 8);
			rsp[i] |= (uint32_t)in[t] << (uint32_t)shift;
			
			/*
			INV_MSG(INV_MSG_LEVEL_INFO, "spi cmd: rw=0x%x, addr=0x%02x, status=0x%x, data=0x%04x, crc=0x%02x\n",
			(int)(cmd[i] >> 31) & 0x1,
			(int)(cmd[i] >> 26) & 0x1f,
			(int)(cmd[i] >> 24) & 0x3,
			(int)(cmd[i] >> 8) & 0xffff,
			(int)(cmd[i] >> 0) & 0xff);
			INV_MSG(INV_MSG_LEVEL_INFO, "spi rsp: rw=0x%x, addr=0x%02x, status=0x%x, data=0x%04x, crc=0x%02x\n",
			(int)(rsp[i] >> 31) & 0x1,
			(int)(rsp[i] >> 26) & 0x1f,
			(int)(rsp[i] >> 24) & 0x3,
			(int)(rsp[i] >> 8) & 0xffff,
			(int)(rsp[i] >> 0) & 0xff);
			*/
		}
	}

	// return INV_ERROR_SUCCESS;
	
}
inv_dev_config_t *device_config;
inv_dev_config_t dev_config;
bool AP_InertialSensor_IIM4623x::init()
{
   
    _clip_limit = 7.5f * GRAVITY_MSS;
    // gyro_scale = radians(0.1);
    expected_sample_rate_hz = 8000;
    WITH_SEMAPHORE(dev->get_semaphore());
    dev->set_device_type(DEVTYPE_INS_IIM46234);

    

	dev_config.accel_fs_sel  = ACCEL_FS_SEL;
	dev_config.gyro_fs_sel   = GYRO_FS_SEL;
	dev_config.x_axis_filter = X_AXIS_FILTER;
	dev_config.y_axis_filter = Y_AXIS_FILTER;
	dev_config.z_axis_filter = Z_AXIS_FILTER;

    setup_iam();


    return true;
}

void AP_InertialSensor_IIM4623x::inv_iam20685_read_reg(struct inv_iam20685 *s, uint8_t offset, uint16_t *data)
{
	//assert(s);

	uint32_t cmd, rsp;


	// Check if offset has only 5 bits
	// if ((offset >> 5) != 0) {
	// 	return INV_ERROR_BAD_ARG;
	// }

	// Send command
	cmd = build_cmd(offset, 0, INV_REG_R);
	send_cmd(s, &cmd, &rsp, 1);

	// uint32_t dummy = 0xAD00007C;
	// // Send dummy command to read back response.
	// send_cmd(s, &dummy, &rsp, 1);
                                                
	// Get data
	*data = (rsp >> 8) & 0xFFFF;
	/*
	 * If the response has an error but passes the crc check, it implies that
	 * there is an internal error in the sensor and that the alarm bits should
	 * be read
	 */


}


void AP_InertialSensor_IIM4623x::inv_iam20685_write_reg(struct inv_iam20685 *s, uint8_t offset, const uint16_t *data)
{
	//assert(s);

	uint32_t cmd, rsp;


	// Check if offset has only 5 bits

	// Send command
	cmd = build_cmd(offset, *data, INV_REG_W);
	send_cmd(s, &cmd, &rsp, 1);


	// Check CRC
	// check_crc(rsp);

	// uint32_t dummy = 0xAD00007C;
	// // Send dummy command to read back response.
	// send_cmd(s, &dummy, &rsp, 1);


	// Check CRC and RS
	// check_crc(rsp);

	// check_return_status(rsp);
	// if (ret != INV_ERROR_SUCCESS)
	// 	return ret;

}
void AP_InertialSensor_IIM4623x::inv_iam20685_select_bank(struct inv_iam20685 *s, uint16_t bank)
{
	

	inv_iam20685_write_reg(s, IAM20685_REG_BANK_SELECT_OFFSET, &bank);

	
}


void AP_InertialSensor_IIM4623x::inv_iam20685_set_accel_fs_sel(struct inv_iam20685 *s, uint8_t accel_fs_sel)
{
	uint16_t data;
	 

	inv_iam20685_select_bank(s, IAM20685_REG_ACCEL_FS_SEL_BANK);

	inv_iam20685_read_reg(s, IAM20685_REG_ACCEL_FS_SEL_OFFSET, &data);
	data &= ~IAM20685_REG_ACCEL_FS_SEL_MASK;
	data |= accel_fs_sel << IAM20685_REG_ACCEL_FS_SEL_POS;
	inv_iam20685_write_reg(s, IAM20685_REG_ACCEL_FS_SEL_OFFSET, &data);

	inv_iam20685_select_bank(s, 0);

	 
}

void AP_InertialSensor_IIM4623x::inv_iam20685_get_accel_fs_sel(struct inv_iam20685 *s, uint8_t *accel_fs_sel)
{
	uint16_t data;
	 

	inv_iam20685_select_bank(s, IAM20685_REG_ACCEL_FS_SEL_BANK);

	inv_iam20685_read_reg(s, IAM20685_REG_ACCEL_FS_SEL_OFFSET, &data);
	*accel_fs_sel = (data & IAM20685_REG_ACCEL_FS_SEL_MASK) >> IAM20685_REG_ACCEL_FS_SEL_POS;

	inv_iam20685_select_bank(s, 0);

	 
}

void AP_InertialSensor_IIM4623x::inv_iam20685_set_gyro_fs_sel(struct inv_iam20685 *s, uint8_t gyro_fs_sel)
{
	uint16_t data;
	 

	inv_iam20685_select_bank(s, IAM20685_REG_GYRO_FS_SEL_BANK);

	inv_iam20685_read_reg(s, IAM20685_REG_GYRO_FS_SEL_OFFSET, &data);
	data &= ~IAM20685_REG_GYRO_FS_SEL_MASK;
	data |= gyro_fs_sel << IAM20685_REG_GYRO_FS_SEL_POS;
	inv_iam20685_write_reg(s, IAM20685_REG_GYRO_FS_SEL_OFFSET, &data);

	inv_iam20685_select_bank(s, 0);

	 
}

void AP_InertialSensor_IIM4623x::inv_iam20685_get_gyro_fs_sel(struct inv_iam20685 *s, uint8_t *gyro_fs_sel)
{
	uint16_t data;
	 

	inv_iam20685_select_bank(s, IAM20685_REG_GYRO_FS_SEL_BANK);

	inv_iam20685_read_reg(s, IAM20685_REG_GYRO_FS_SEL_OFFSET, &data);
	*gyro_fs_sel = (data & IAM20685_REG_GYRO_FS_SEL_MASK) >> IAM20685_REG_GYRO_FS_SEL_POS;

	inv_iam20685_select_bank(s, 0);

	 
}

void AP_InertialSensor_IIM4623x::inv_iam20685_set_flt_y(struct inv_iam20685 *s, uint8_t *flt_y)
{
	uint16_t data;
	 

	inv_iam20685_read_reg(s, IAM20685_REG_FLT_Y_OFFSET, &data);
	data &= ~IAM20685_REG_FLT_Y_MASK;
	data |= *flt_y << IAM20685_REG_FLT_Y_POS;
	inv_iam20685_write_reg(s, IAM20685_REG_FLT_Y_OFFSET, &data);

	 
}

void AP_InertialSensor_IIM4623x::inv_iam20685_set_flt_z(struct inv_iam20685 *s, uint8_t *flt_z)
{
	uint16_t data;
	 

	inv_iam20685_read_reg(s, IAM20685_REG_FLT_Z_OFFSET, &data);
	data &= ~IAM20685_REG_FLT_Z_MASK;
	data |= *flt_z << IAM20685_REG_FLT_Z_POS;
	inv_iam20685_write_reg(s, IAM20685_REG_FLT_Z_OFFSET, &data);

	 
}

void AP_InertialSensor_IIM4623x::inv_iam20685_set_flt_x(struct inv_iam20685 *s, uint8_t *flt_x)
{
	uint16_t data;
	 

	inv_iam20685_read_reg(s, IAM20685_REG_FLT_X_OFFSET, &data);
	data &= ~IAM20685_REG_FLT_X_MASK;
	data |= *flt_x << IAM20685_REG_FLT_X_POS;
	inv_iam20685_write_reg(s, IAM20685_REG_FLT_X_OFFSET, &data);

	 
}

void AP_InertialSensor_IIM4623x::inv_iam20685_get_whoami(struct inv_iam20685 *s, uint16_t *whoami)
{
	

	inv_iam20685_select_bank(s, IAM20685_REG_WHOAMI_BANK);
	inv_iam20685_read_reg(s, IAM20685_REG_WHOAMI_OFFSET, whoami);
	inv_iam20685_select_bank(s, 0);

}

void AP_InertialSensor_IIM4623x:: inv_iam20685_routeODRclock_to_pin12(struct inv_iam20685 *s)
{
	uint16_t data;

	inv_iam20685_select_bank(s, IAM20685_REG_ODR_CONFIG_1_BANK);

	// Set BANK3, register address 0x14, bit 9 to 1b
	inv_iam20685_read_reg(s, IAM20685_REG_ODR_CONFIG_4_OFFSET, &data);
	data &= ~IAM20685_REG_ODR_CONFIG_4_MASK;
	data |= 1 << IAM20685_REG_ODR_CONFIG_4_POS;
	inv_iam20685_write_reg(s, IAM20685_REG_ODR_CONFIG_4_OFFSET, &data);

	// Set BANK3, register address 0x17, bit 12 to 1b
	inv_iam20685_read_reg(s, IAM20685_REG_ODR_CONFIG_6_OFFSET, &data);
	data &= ~IAM20685_REG_ODR_CONFIG_6_MASK;
	data |= 1 << IAM20685_REG_ODR_CONFIG_6_POS;
	inv_iam20685_write_reg(s, IAM20685_REG_ODR_CONFIG_6_OFFSET, &data);

	// Set BANK3, register address 0x11, bits 13:8 to 0x21
	inv_iam20685_read_reg(s, IAM20685_REG_ODR_CONFIG_1_OFFSET, &data);
	data &= ~IAM20685_REG_ODR_CONFIG_1_MASK;
	data |= 0x21 << IAM20685_REG_ODR_CONFIG_1_POS;
	inv_iam20685_write_reg(s, IAM20685_REG_ODR_CONFIG_1_OFFSET, &data);

	// Set BANK3, register address 0x13, bits 7:4 to 0x08
	inv_iam20685_read_reg(s, IAM20685_REG_ODR_CONFIG_2_OFFSET, &data);
	data &= ~IAM20685_REG_ODR_CONFIG_2_MASK;
	data |= 0x08 << IAM20685_REG_ODR_CONFIG_2_POS;
	inv_iam20685_write_reg(s, IAM20685_REG_ODR_CONFIG_2_OFFSET, &data);

	// Set BANK3, register address 0x14, bit 5 to 1b
	inv_iam20685_read_reg(s, IAM20685_REG_ODR_CONFIG_3_OFFSET, &data);
	data &= ~IAM20685_REG_ODR_CONFIG_3_MASK;
	data |= 1 << IAM20685_REG_ODR_CONFIG_3_POS;
	inv_iam20685_write_reg(s, IAM20685_REG_ODR_CONFIG_3_OFFSET, &data);

	// Set BANK3, register address 0x16, bit 0 to 1b
	inv_iam20685_read_reg(s, IAM20685_REG_ODR_CONFIG_5_OFFSET, &data);
	data &= ~IAM20685_REG_ODR_CONFIG_5_MASK;
	data |= 1 << IAM20685_REG_ODR_CONFIG_5_POS;
	inv_iam20685_write_reg(s, IAM20685_REG_ODR_CONFIG_5_OFFSET, &data);

	// inv_iam20685_select_bank(s, 0);

}

void AP_InertialSensor_IIM4623x::inv_iam20685_set_hard_reset(struct inv_iam20685 *s)
{
	uint32_t cmd, rsp;

	cmd = build_cmd(IAM20685_REG_HARD_RESET_OFFSET, IAM20685_REG_HARD_RESET_MASK, INV_REG_W);
	send_cmd(s, &cmd, &rsp, 1);
}
void AP_InertialSensor_IIM4623x::inv_iam20685_get_fixed_value(struct inv_iam20685 *s, uint16_t *fixed_value)
{
	return inv_iam20685_read_reg(s, IAM20685_REG_FIXED_VALUE_OFFSET, fixed_value);
}

void AP_InertialSensor_IIM4623x::inv_iam20685_unlock_chip(struct inv_iam20685 *s)
{
	
	uint32_t cmd[8];
	uint32_t rsp[7] = { 0 };
/*
	cmd[0] = 0xE4000288;
	cmd[1] = 0xE400018B;
	cmd[2] = 0xE400048E;
	cmd[3] = 0xE40300AD;
	cmd[4] = 0xE4018017;
	cmd[5] = 0xE4028030;
	*/
	cmd[0] = dummy_cmd;
	cmd[1] = 0x880200E4;
	cmd[2] = 0x8B0100E4;
	cmd[3] = 0x8E0400E4;
	cmd[4] = 0xAD0003E4;
	cmd[5] = 0x178001E4;
	cmd[6] = 0x308002E4;
	cmd[7] = dummy_cmd;

	send_cmd(s, cmd, rsp, 8);
	

}
void AP_InertialSensor_IIM4623x::inv_iam20685_release_capture_mode(struct inv_iam20685 *s)
{
	uint16_t data;


	inv_iam20685_read_reg(s, IAM20685_REG_CAPTURE_MODE_OFFSET, &data);
	data &= ~IAM20685_REG_CAPTURE_MODE_MASK;
	inv_iam20685_write_reg(s, IAM20685_REG_CAPTURE_MODE_OFFSET, &data);

}

void AP_InertialSensor_IIM4623x::inv_iam20685_set_capture_mode(struct inv_iam20685 *s)
{
	uint16_t data;


	inv_iam20685_read_reg(s, IAM20685_REG_CAPTURE_MODE_OFFSET, &data);
	data |= IAM20685_REG_CAPTURE_MODE_MASK;
	inv_iam20685_write_reg(s, IAM20685_REG_CAPTURE_MODE_OFFSET, &data);

}

void AP_InertialSensor_IIM4623x::GetSensorData(void)
{
    WITH_SEMAPHORE(dev->get_semaphore());
	int16_t  accel_x, accel_y, accel_z;
	int16_t  gyro_x, gyro_y, gyro_z;
	float  faccel_x, faccel_y, faccel_z;
	float  fgyro_x, fgyro_y, fgyro_z;
	int16_t  temp1;
	float ftemp1;
	// uint32_t timestamp = 0;

	uint32_t cmd[8];
	uint32_t rsp[8] = { 0 };
	// static uint8_t accel_identical_cntr=0;
	// static uint8_t gyro_identical_cntr = 0;

	/* Read Timestamp from ring-buffer */
	//timestamp = timer_get_irq_timestamp_us();

	inv_iam20685_set_capture_mode(&icm_device);

	/* send 8 command contiguously to read gyro, accel and temp1 data */
	cmd[0] = build_cmd(IAM20685_REG_GYRO_X_DATA_OFFSET, 0, INV_REG_R);
	cmd[1] = build_cmd(IAM20685_REG_GYRO_Y_DATA_OFFSET, 0, INV_REG_R);
	cmd[2] = build_cmd(IAM20685_REG_GYRO_Z_DATA_OFFSET, 0, INV_REG_R);
	cmd[3] = build_cmd(IAM20685_REG_ACCEL_X_DATA_OFFSET, 0, INV_REG_R);
	cmd[4] = build_cmd(IAM20685_REG_ACCEL_Y_DATA_OFFSET, 0, INV_REG_R);
	cmd[5] = build_cmd(IAM20685_REG_ACCEL_Z_DATA_OFFSET, 0, INV_REG_R);
	cmd[6] = build_cmd(IAM20685_REG_TEMP1_DATA_OFFSET, 0, INV_REG_R);
	cmd[7] = dummy_cmd;

	send_cmd(&icm_device, cmd, rsp, 8);


	gyro_x  = (rsp[0] >> 8) & (uint32_t)0xffff;
	gyro_y  = (rsp[1] >> 8) & (uint32_t)0xffff;
	gyro_z  = (rsp[2] >> 8) & (uint32_t)0xffff;
	accel_x = (rsp[3] >> 8) & (uint32_t)0xffff;
	accel_y = (rsp[4] >> 8) & (uint32_t)0xffff;
	accel_z = (rsp[5] >> 8) & (uint32_t)0xffff;
	temp1   = (rsp[6] >> 8) & (uint32_t)0xffff;
	ftemp1 = 25.0 + temp1/20;
	

	float accel_scale = 9.8/(pow(2,15)/32);

	float gyro_scale = DEG_TO_RAD/(pow(2,15)/1966);

	fgyro_x = ((float)gyro_x) * gyro_scale;
	fgyro_y = ((float)gyro_y) * gyro_scale;
	fgyro_z = ((float)gyro_z) * gyro_scale;	

	faccel_x = ((float)accel_x) * accel_scale;
	faccel_y = ((float)accel_y) * accel_scale;
	faccel_z = ((float)accel_z) * accel_scale;
	// GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IAM20685: gyro_xyz %f, %f, %f, accel_xyz %f, %f, %f temp %f \n",fgyro_x, fgyro_y, fgyro_z, faccel_x, faccel_y, faccel_z, ftemp1 );
    uint64_t timestamp=0;

    if (first_timestamp_us == 0) {
        first_timestamp_us = AP_HAL::micros64();
        first_imu_timestamp_us = timestamp;
    }
	
	Vector3f accel{faccel_x, faccel_y, faccel_z};
    Vector3f gyro{fgyro_x, fgyro_y, fgyro_z};
	
    const uint64_t sample_us = (timestamp - first_imu_timestamp_us) + first_timestamp_us;
    _rotate_and_correct_accel(accel_instance, accel);
    _notify_new_accel_raw_sample(accel_instance, accel, sample_us);

    _rotate_and_correct_gyro(gyro_instance, gyro);
    _notify_new_gyro_raw_sample(gyro_instance, gyro, sample_us);

    /*
      publish average temperature at 20Hz
     */
    temp_sum += ftemp1;
    temp_count++;

    if (temp_count == 100) {
        _publish_temperature(accel_instance, temp_sum/temp_count);
        temp_sum = 0;
        temp_count = 0;
    }

	inv_iam20685_release_capture_mode(&icm_device);

}
void AP_InertialSensor_IIM4623x::setup_iam(void){
	WITH_SEMAPHORE(dev->get_semaphore());
    
    // dummy_cmd = build_cmd(IAM20685_REG_BANK_SELECT_OFFSET, 0, INV_REG_R);

    inv_iam20685_set_hard_reset(&icm_device);
    hal.scheduler->delay(300);
	// inv_iam20685_sleep_ms(300);

	/* Unlock the chip */
	inv_iam20685_unlock_chip(&icm_device);


    /* enable DRDY */
    //inv_iam20685_routeODRclock_to_pin12(&icm_device);
	

	/* Check Fixed Value */
	uint16_t fixed_value;
	inv_iam20685_get_fixed_value(&icm_device, &fixed_value);
	// INV_MSG(INV_MSG_LEVEL_INFO, "fixed_value = 0x%x", fixed_value);
    
	/* Check WHOAMI value */
	uint16_t who_am_i;
	inv_iam20685_get_whoami(&icm_device, &who_am_i);
	// INV_MSG(INV_MSG_LEVEL_INFO, "whoami = 0x%x", who_am_i);
	GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IAM20685: WHOAMI %d \n", who_am_i);


    // 	/* Configure accel and gyro FSR and filter settings */
	inv_iam20685_set_accel_fs_sel(&icm_device, (device_config->accel_fs_sel));
	inv_iam20685_set_gyro_fs_sel(&icm_device, (device_config->gyro_fs_sel));

	inv_iam20685_set_flt_x(&icm_device, &(device_config->x_axis_filter));
	inv_iam20685_set_flt_y(&icm_device, &(device_config->y_axis_filter));
	inv_iam20685_set_flt_z(&icm_device, &(device_config->z_axis_filter));
}


/*
  sensor read loop
 */
void AP_InertialSensor_IIM4623x::loop(void)
{
    while(true){
        GetSensorData();
    }
    
	


}

bool AP_InertialSensor_IIM4623x::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}