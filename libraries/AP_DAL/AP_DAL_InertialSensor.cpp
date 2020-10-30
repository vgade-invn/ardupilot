#include "AP_DAL_InertialSensor.h"

#include <AP_Logger/AP_Logger.h>

AP_DAL_InertialSensor::AP_DAL_InertialSensor()
{
    for (uint8_t i=0; i<ARRAY_SIZE(_RISI); i++) {
        _RISI[i].head1 = HEAD_BYTE1;
        _RISI[i].head2 = HEAD_BYTE2;
        _RISI[i].msgid = LOG_RISI_MSG;
        _RISI[i].instance = i;
        _RISJ[i].head1 = HEAD_BYTE1;
        _RISJ[i].head2 = HEAD_BYTE2;
        _RISJ[i].msgid = LOG_RISJ_MSG;
        _RISJ[i].instance = i;
    }
}

void AP_DAL_InertialSensor::start_frame(const uint64_t time_us)
{
    const auto &ins = AP::ins();

    auto &logger = AP::logger();

    _RISH.time_us = time_us;
    _RISH.loop_rate_hz = ins.get_loop_rate_hz();
    _RISH.primary_gyro = ins.get_primary_gyro();
    _RISH.loop_delta_t = ins.get_loop_delta_t();
    _RISH.primary_accel = ins.get_primary_accel();
    _RISH.accel_count = ins.get_accel_count();
    _RISH.gyro_count = ins.get_gyro_count();
    _RISH.last_update_usec = ins.get_last_update_usec();
    logger.WriteReplayBlock(&_RISH, sizeof(_RISH));

    for (uint8_t i=0; i<ARRAY_SIZE(_RISI); i++) {
        log_RISI &RISI = _RISI[i];

        RISI.time_us = time_us;

        // accel stuff
        RISI.use_accel = ins.use_accel(i);
        RISI.accel.from_Vector3f(ins.get_accel(i));

        Vector3f v;
        RISI.get_delta_velocity_ret = ins.get_delta_velocity(i, v);
        RISI.delta_velocity.from_Vector3f(v);

        RISI.delta_velocity_dt = ins.get_delta_velocity_dt(i);

        logger.WriteReplayBlock(&RISI, sizeof(RISI));

        // gryo stuff
        log_RISJ &RISJ = _RISJ[i];
        RISJ.time_us = time_us;
        RISJ.use_gyro = ins.use_gyro(i);
        RISJ.gyro.from_Vector3f(ins.get_gyro(i));

        RISJ.delta_angle_dt = ins.get_delta_angle_dt(i);
        RISJ.get_delta_angle_ret = ins.get_delta_angle(i, v);
        RISJ.delta_angle.from_Vector3f(v);

        logger.WriteReplayBlock(&RISJ, sizeof(RISJ));
    }
}
