#include "AP_InertialSensor.h"
#include <GCS_MAVLink/GCS.h>

// Class level parameters
const AP_Param::GroupInfo AP_InertialSensor::BatchSampler::var_info[] = {
    // @Param: GYR_CNT
    // @DisplayName: DataFlash sample count for gyros
    // @Description: Number of samples to take when logging streams of IMU sensor readings.  Will be rounded down to a multiple of 32.
    // @User: Advanced
    AP_GROUPINFO("GYR_COUNT",  1, AP_InertialSensor::BatchSampler, _gyr_count,   1024),

    // @Param: ACC_CNT
    // @DisplayName: DataFlash sample count for accelerometers
    // @Description: Number of samples to take when logging streams of IMU sensor readings.  Will be rounded down to a multiple of 32.
    // @User: Advanced
    // @Range: 0 65536
    AP_GROUPINFO("ACC_COUNT",  2, AP_InertialSensor::BatchSampler, _acc_count,   1024),

    AP_GROUPEND
};


extern const AP_HAL::HAL& hal;
void AP_InertialSensor::BatchSampler::init()
{
    uint16_t max_samples = MAX((uint16_t)_acc_count, (uint16_t)_gyr_count);
    max_samples -= max_samples % 32; // round down to nearest multiple of 32

    data_x = (int16_t*)calloc(max_samples, sizeof(int16_t));
    data_y = (int16_t*)calloc(max_samples, sizeof(int16_t));
    data_z = (int16_t*)calloc(max_samples, sizeof(int16_t));
    if (data_x == nullptr || data_y == nullptr || data_z == nullptr) {
        free(data_x);
        free(data_y);
        free(data_z);
        data_x = nullptr;
        data_y = nullptr;
        data_z = nullptr;
        //gcs().send_text(MAV_SEVERITY_WARNING, "Failed to allocate %u bytes for IMU batch sampling", total_allocation);
        return;
    }

    //gcs().send_text(MAV_SEVERITY_WARNING, "INS: after allocation %u bytes free", hal.util->available_memory());
    initialised = true;
}

uint16_t AP_InertialSensor::BatchSampler::required_sample_count() const
{
    switch(type) {
    case IMU_SENSOR_TYPE_GYRO:
        return (uint16_t)_gyr_count;
    case IMU_SENSOR_TYPE_ACCEL:
        return (uint16_t)_acc_count;
    }
    // not reached...
    return (uint16_t)_gyr_count;
}

void AP_InertialSensor::BatchSampler::push_data_to_log(uint64_t sample_us)
{
    if (!initialised) {
        return;
    }
    if (data_write_offset - data_read_offset < samples_per_msg) {
        // insuffucient data to pack a packet
        return;
    }
    const uint64_t now = AP_HAL::micros64();
    if (last_sent_ms - now < push_interval_ms) {
        // avoid flooding DataFlash's buffer
        return;
    }
    DataFlash_Class *dataflash = DataFlash_Class::instance();
    if (dataflash == nullptr) {
        // should not have been called
        return;
    }

    // possibly send isb header:
    if (!isbh_sent && data_read_offset == 0) {
        float sample_rate = 0; // avoid warning about uninitialised values
        switch(type) {
        case IMU_SENSOR_TYPE_GYRO:
            sample_rate = _imu._gyro_raw_sample_rates[instance];
            break;
        case IMU_SENSOR_TYPE_ACCEL:
            sample_rate = _imu._accel_raw_sample_rates[instance];
            break;
        }
        if (!dataflash->Log_Write_ISBH(isb_seqnum,
                                       type,
                                       instance,
                                       multiplier,
                                       sample_us,
                                       sample_rate)) {
            // buffer full?
            return;
        }
        isbh_sent = true;
    }
    // pack and send a data packet:
    if (!dataflash->Log_Write_ISBD(isb_seqnum,
                                   data_read_offset/samples_per_msg,
                                   &data_x[data_read_offset],
                                   &data_y[data_read_offset],
                                   &data_z[data_read_offset])) {
        // maybe later?!
        return;
    }
    data_read_offset += samples_per_msg;
    last_sent_ms = AP_HAL::millis();
    if (data_read_offset >= required_sample_count()) {
        // that was the last one.  Clean up:
        data_read_offset = 0;
        isb_seqnum++;
        isbh_sent = false;
        // rotate to next instance:
        instance++;
        switch(type) {
        case IMU_SENSOR_TYPE_ACCEL:
            if (instance >= _imu._accel_count) {
                type = IMU_SENSOR_TYPE_GYRO;
                instance = 0;
                multiplier = 10000;
            }
            break;
        case IMU_SENSOR_TYPE_GYRO:
            if (instance >= _imu._gyro_count) {
                type = IMU_SENSOR_TYPE_ACCEL;
                instance = 0;
                multiplier = 1000;
            }
            break;
        }
        data_write_offset = 0; // unlocks writing process
    }
}

bool AP_InertialSensor::BatchSampler::should_log(uint8_t _instance, IMU_SENSOR_TYPE _type)
{
    if (!initialised) {
        return false;
    }
    // TODO: disable based on IMU_SENSORB_COUNT == 0
    if (_instance != instance) {
        return false;
    }
    if (_type != type) {
        return false;
    }
    if (data_write_offset >= required_sample_count()) {
        return false;
    }
    return true;
}

void AP_InertialSensor::BatchSampler::sample(uint8_t _instance, AP_InertialSensor::IMU_SENSOR_TYPE _type, uint64_t sample_us, const Vector3f &_sample)
{
    if (!should_log(_instance, _type)) {
        push_data_to_log(sample_us);
        return;
    }

    data_x[data_write_offset] = multiplier*_sample.x;
    data_y[data_write_offset] = multiplier*_sample.y;
    data_z[data_write_offset] = multiplier*_sample.z;

    data_write_offset++; // may unblock the reading process

    push_data_to_log(sample_us);
}
