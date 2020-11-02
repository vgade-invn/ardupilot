#include "AP_DAL_InertialSensor.h"

#include <AP_Logger/AP_Logger.h>

AP_DAL_InertialSensor::AP_DAL_InertialSensor()
{
    for (uint8_t i=0; i<ARRAY_SIZE(_RISI); i++) {
        _RISI[i].instance = i;
        _RISJ[i].instance = i;
    }

}

void AP_DAL_InertialSensor::start_frame()
{
    const auto &ins = AP::ins();

    const log_RISH old_RISH = _RISH;

    _RISH.loop_rate_hz = ins.get_loop_rate_hz();
    _RISH.primary_gyro = ins.get_primary_gyro();
    _RISH.loop_delta_t = ins.get_loop_delta_t();
    _RISH.primary_accel = ins.get_primary_accel();
    _RISH.accel_count = ins.get_accel_count();
    _RISH.gyro_count = ins.get_gyro_count();
    if (STRUCT_NEQ(old_RISH, _RISH)) {
        WRITE_REPLAY_BLOCK(RISH, _RISH);
    }

    for (uint8_t i=0; i<ARRAY_SIZE(_RISI); i++) {
        log_RISI &RISI = _RISI[i];
        const log_RISI old_RISI = RISI;

        // accel stuff
        RISI.use_accel = ins.use_accel(i);
        RISI.accel = ins.get_accel(i);

        RISI.get_delta_velocity_ret = ins.get_delta_velocity(i, RISI.delta_velocity);

        RISI.delta_velocity_dt = ins.get_delta_velocity_dt(i);

        if (STRUCT_NEQ(RISI, old_RISI)) {
            WRITE_REPLAY_BLOCK(RISI, RISI);
        }

        // gryo stuff
        log_RISJ &RISJ = _RISJ[i];
        const log_RISJ old_RISJ = RISJ;
        RISJ.use_gyro = ins.use_gyro(i);
        RISJ.gyro = ins.get_gyro(i);

        RISJ.delta_angle_dt = ins.get_delta_angle_dt(i);
        RISJ.get_delta_angle_ret = ins.get_delta_angle(i, RISJ.delta_angle);

        if (STRUCT_NEQ(RISJ, old_RISJ)) {
            WRITE_REPLAY_BLOCK(RISJ, RISJ);
        }

        // update sensor position
        pos[i] = ins.get_imu_pos_offset(i);
    }
}
