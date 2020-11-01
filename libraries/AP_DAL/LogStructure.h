#pragma once

#include <AP_Logger/LogStructure.h>
#include <AP_Math/vector3.h>

#define LOG_IDS_FROM_DAL \
    LOG_RFRH_MSG, \
    LOG_RFRF_MSG, \
    LOG_REV2_MSG, \
    LOG_RSO2_MSG, \
    LOG_RWA2_MSG, \
    LOG_REV3_MSG, \
    LOG_RSO3_MSG, \
    LOG_RWA3_MSG, \
    LOG_REY3_MSG, \
    LOG_RFRN_MSG, \
    LOG_RISH_MSG, \
    LOG_RISI_MSG, \
    LOG_RISJ_MSG, \
    LOG_RBRH_MSG, \
    LOG_RBRI_MSG, \
    LOG_RRNH_MSG, \
    LOG_RRNI_MSG, \
    LOG_RGPH_MSG, \
    LOG_RGPI_MSG, \
    LOG_RGPJ_MSG, \
    LOG_RASH_MSG, \
    LOG_RASI_MSG, \
    LOG_RBCH_MSG, \
    LOG_RBCI_MSG, \
    LOG_RVOH_MSG, \
    LOG_RMGH_MSG, \
    LOG_RMGI_MSG

// Replay Data Structures
struct log_RFRH {
    uint64_t time_us;
    uint32_t frame_number;
    float trimx;
    float trimy;
    float trimz;
    float EAS2TAS;
    uint32_t available_memory;
    uint16_t time_flying_ms;
    uint8_t state_bitmask;
    uint8_t rangefinder_ptr_is_null;
    uint8_t get_compass_is_null;
    uint8_t airspeed_ptr_is_null;
    uint8_t fly_forward;
    uint8_t vehicle_class;
    uint8_t ahrs_airspeed_sensor_enabled;
    uint8_t ekf_type;
    uint8_t _end;
};

struct log_RFRN {
    uint64_t time_us;
    int32_t lat;
    int32_t lng;
    int32_t alt;
    uint8_t _end;
};

struct log_RFRF {
    uint64_t time_us;
    uint8_t frame_type;
    uint8_t _end;
};

// Replay Data Structure - Inertial Sensor header
struct log_RISH {
    uint64_t time_us;
    uint16_t loop_rate_hz;
    uint8_t primary_gyro;
    uint8_t primary_accel;
    float loop_delta_t;
    uint32_t last_update_usec;
    uint8_t accel_count;
    uint8_t gyro_count;
    uint8_t _end;
};

// Replay Data Structure - Inertial Sensor instance data
struct log_RISI {
    uint64_t time_us;
    Vector3f pos;
    Vector3f accel;
    Vector3f delta_velocity;
    float delta_velocity_dt;
    uint8_t use_accel;
    uint8_t get_delta_velocity_ret;
    uint8_t instance;
    uint8_t _end;
};

// Replay Data Structure - Inertial Sensor instance data
struct log_RISJ {
    uint64_t time_us;
    Vector3f gyro;
    Vector3f delta_angle;
    float delta_angle_dt;
    uint8_t use_gyro;
    uint8_t get_delta_angle_ret;
    uint8_t instance;
    uint8_t _end;
};

// @LoggerMessage: REV2
// @Description: Replay Event
struct log_REV2 {
    uint64_t time_us;
    uint8_t event;
    uint8_t _end;
};

// @LoggerMessage: RSO2
// @Description: Replay Set Origin event
struct log_RSO2 {
    uint64_t time_us;
    int32_t lat;
    int32_t lng;
    int32_t alt;
    uint8_t _end;
};

// @LoggerMessage: RWA2
// @Description: Replay set-default-airspeed event
struct log_RWA2 {
    uint64_t time_us;
    float airspeed;
    uint8_t _end;
};

// @LoggerMessage: REV3
// @Description: Replay Event
struct log_REV3 {
    uint64_t time_us;
    uint8_t event;
    uint8_t _end;
};

// @LoggerMessage: RSO3
// @Description: Replay Set Origin event
struct log_RSO3 {
    uint64_t time_us;
    int32_t lat;
    int32_t lng;
    int32_t alt;
    uint8_t _end;
};

// @LoggerMessage: RWA3
// @Description: Replay set-default-airspeed event
struct log_RWA3 {
    uint64_t time_us;
    float airspeed;
    uint8_t _end;
};

// @LoggerMessage: REY3
// @Description: Replay Euler Yaw event
struct log_REY3 {
    uint64_t time_us;
    float yawangle;
    float yawangleerr;
    uint32_t timestamp_ms;
    uint8_t type;
    uint8_t _end;
};

// @LoggerMessage: RBRH
// @Description: Replay Data Barometer Header
struct log_RBRH {
    uint64_t time_us;
    uint8_t primary;
    uint8_t num_instances;
    uint8_t _end;
};

// @LoggerMessage: RBRI
// @Description: Replay Data Barometer Instance
struct log_RBRI {
    uint64_t time_us;
    uint32_t last_update_ms;
    float altitude;  // from get_altitude
    bool healthy;
    uint8_t instance;
    uint8_t _end;
};

// @LoggerMessage: RRNH
// @Description: Replay Data Rangefinder Header
struct log_RRNH {
    uint64_t time_us;
    // this is rotation-pitch-270!
    int16_t ground_clearance_cm;
    int16_t max_distance_cm;
    uint8_t _end;
};

// @LoggerMessage: RRNI
// @Description: Replay Data Rangefinder Instance
struct log_RRNI {
    uint64_t time_us;
    Vector3f pos_offset;
    uint16_t distance_cm;
    uint8_t orientation;
    uint8_t status;
    uint8_t instance;
    uint8_t _end;
};

// @LoggerMessage: RGPH
// @Description: Replay Data GPS Header
struct log_RGPH {
    uint64_t time_us;
    uint8_t num_sensors;
    uint8_t primary_sensor;
    uint8_t _end;
};

// @LoggerMessage: RGPI
// @Description: Replay Data GPS Instance
struct log_RGPI {
    uint64_t time_us;
    uint32_t last_message_time_ms;
    int32_t lat;
    int32_t lng;
    int32_t alt;
    float hacc;
    float vacc;
    float lag_sec;
    uint16_t hdop;
    uint8_t status;
    uint8_t have_vertical_velocity;
    uint8_t horizontal_accuracy_returncode;
    uint8_t vertical_accuracy_returncode;
    uint8_t num_sats;
    uint8_t get_lag_returncode;
    uint8_t instance;
    uint8_t _end;
};

// @LoggerMessage: RGPJ
// @Description: Replay Data GPS Instance - more data
struct log_RGPJ {
    uint64_t time_us;
    Vector3f velocity;
    uint32_t speed_accuracy_returncode;
    float sacc;
    Vector3f antenna_offset;
    float yaw_deg;
    float yaw_accuracy_deg;
    uint8_t gps_yaw_deg_returncode;
    uint8_t instance;
    uint8_t _end;
};

// Replay Data Structure - Airspeed Sensor header
struct log_RASH {
    uint64_t time_us;
    uint8_t num_sensors;
    uint8_t primary;
    uint8_t _end;
};

// Replay Data Structure - Airspeed Sensor instance
struct log_RASI {
    uint64_t time_us;
    float airspeed;
    uint32_t last_update_ms;
    bool healthy;
    bool use;
    uint8_t instance;
    uint8_t _end;
};

// @LoggerMessage: RMGH
// @Description: Replay Data Magnetometer Header
struct log_RMGH {
    uint64_t time_us;
    float declination;
    uint8_t count;
    bool auto_declination_enabled;
    uint8_t num_enabled;
    bool learn_offsets_enabled;
    bool consistent;
    uint8_t _end;
};

// @LoggerMessage: RMGI
// @Description: Replay Data Magnetometer Instance
struct log_RMGI {
    uint64_t time_us;
    uint32_t last_update_usec;
    Vector3f offsets;
    Vector3f field;
    bool use_for_yaw;
    bool healthy;
    bool have_scale_factor;
    uint8_t instance;
    uint8_t _end;
};

// @LoggerMessage: RBCH
// @Description: Replay Data Beacon Header
struct log_RBCH {
    uint64_t time_us;
    Vector3f vehicle_position_ned;
    float accuracy_estimate;
    int32_t origin_lat;
    int32_t origin_lng;
    int32_t origin_alt;
    bool get_vehicle_position_ned_returncode;
    uint8_t count;
    bool get_origin_returncode;
    uint8_t ptr_is_nullptr;
    uint8_t _end;
};

// @LoggerMessage: RBCI
// @Description: Replay Data Beacon Instance
struct log_RBCI {
    uint64_t time_us;
    uint32_t last_update_ms;
    Vector3f position;
    float distance;
    uint8_t healthy;
    uint8_t instance;
    uint8_t _end;
};

// @LoggerMessage: RVOH
// @Description: Replay Data Visual Odometry data
struct log_RVOH {
    uint64_t time_us;
    Vector3f pos_offset;
    uint32_t delay_ms;
    uint8_t healthy;
    bool enabled;
    uint8_t ptr_is_nullptr;
    uint8_t _end;
};

#define RLOG_SIZE(sname) 3+offsetof(struct log_ ##sname,_end)

#define WRITE_REPLAY_BLOCK(sname,v) AP::logger().WriteReplayBlock(LOG_## sname ##_MSG, &v, offsetof(log_ ##sname, _end))

#define LOG_STRUCTURE_FROM_DAL        \
    { LOG_RFRH_MSG, RLOG_SIZE(RFRH),                          \
      "RFRH", "QIffffIHBBBBBBBB", "TimeUS,FN,TX,TY,TZ,E2T,AM,TF,State,NlRF,NlCRP,NlAS,FF,VC,ASE,EKT", "s---------------", "F---------------" }, \
    { LOG_RFRF_MSG, RLOG_SIZE(RFRF),                                   \
      "RFRF", "QB", "TimeUS,FT", "s-", "F-" }, \
    { LOG_RFRN_MSG, RLOG_SIZE(RFRN),                            \
      "RFRN", "QIII", "TimeUS,HLat,HLon,HAlt", "sDUm", "FGGB" }, \
    { LOG_REV2_MSG, RLOG_SIZE(REV2),                                   \
      "REV2", "QB", "TimeUS,Event", "s-", "F-" }, \
    { LOG_RSO2_MSG, RLOG_SIZE(RSO2),                         \
      "RSO2", "QIII", "TimeUS,Lat,Lon,Alt", "sDUm", "FGGB" }, \
    { LOG_RWA2_MSG, RLOG_SIZE(RWA2),                         \
      "RWA2", "Qf", "TimeUS,Airspeed", "sn", "F0" }, \
    { LOG_REV3_MSG, RLOG_SIZE(REV3),                \
      "REV3", "QB", "TimeUS,Event", "s-", "F-" }, \
    { LOG_RSO3_MSG, RLOG_SIZE(RSO3),                         \
      "RSO3", "QIII", "TimeUS,Lat,Lon,Alt", "sDUm", "FGGB" }, \
    { LOG_RWA3_MSG, RLOG_SIZE(RWA3),                         \
      "RWA3", "Qf", "TimeUS,Airspeed", "sn", "F0" }, \
    { LOG_REY3_MSG, RLOG_SIZE(REY3),                                   \
      "REY3", "QffIB", "TimeUS,yawangle,yawangleerr,timestamp_ms,type", "s???-", "F???-" }, \
    { LOG_RISH_MSG, RLOG_SIZE(RISH),                                   \
      "RISH", "QHBBfIBB", "TimeUS,LR,PG,PA,LD,LU,AC,GC", "s-------", "F-------" }, \
    { LOG_RISI_MSG, RLOG_SIZE(RISI),                                   \
      "RISI", "QffffffffffBBB", "TimeUS,PX,PY,PZ,AX,AY,AZ,DVX,DVY,DVZ,DVDT,UA,GDVR,I", "s------------#", "F-------------" }, \
    { LOG_RISJ_MSG, RLOG_SIZE(RISJ),                                   \
      "RISJ", "QfffffffBBB", "TimeUS,GX,GY,GZ,DAX,DAY,DAZ,DADT,UG,GDAR,I", "s---------#", "F----------" }, \
    { LOG_RASH_MSG, RLOG_SIZE(RASH),                                   \
      "RASH", "QBB", "TimeUS,Primary,NumInst", "s--", "F--" },  \
    { LOG_RASI_MSG, RLOG_SIZE(RASI),                                   \
      "RASI", "QfIBBB", "TimeUS,Spd,UpdateMS,H,Use,I", "s----#", "F-----" }, \
    { LOG_RBRH_MSG, RLOG_SIZE(RBRH),                                   \
      "RBRH", "QBB", "TimeUS,Primary,NumInst", "s--", "F--" },  \
    { LOG_RBRI_MSG, RLOG_SIZE(RBRI),                                   \
      "RBRI", "QIfBB", "TimeUS,LastUpdate,Alt,H,I", "s---#", "F----" }, \
    { LOG_RRNH_MSG, RLOG_SIZE(RRNH),                                   \
      "RRNH", "Qhh", "TimeUS,GCl,MaxD", "s??", "F??" },  \
    { LOG_RRNI_MSG, RLOG_SIZE(RRNI),                                   \
      "RRNI", "QfffHBBB", "TimeUS,OX,OY,OZ,Dist,Orient,Status,I", "s------#", "F-------" }, \
    { LOG_RGPH_MSG, RLOG_SIZE(RGPH),                                   \
      "RGPH", "QBB", "TimeUS,NumInst,Primary", "s--", "F--" },  \
    { LOG_RGPI_MSG, RLOG_SIZE(RGPI),                                   \
      "RGPI", "QIiiifffHBBBBBBB", "TimeUS,LMT,lat,lon,alt,ha,va,lg,hdp,st,hvv,harc,varc,ns,lgrc,I", "s--------------#", "F---------------" }, \
    { LOG_RGPJ_MSG, RLOG_SIZE(RGPJ),                                   \
      "RGPJ", "QfffIffffffBB", "TimeUS,vx,vy,vz,sarc,sa,aox,aoy,aoz,yd,yda,ydrc,I", "s-----------#", "F------------" }, \
    { LOG_RMGH_MSG, RLOG_SIZE(RMGH),                                   \
      "RMGH", "QBBfBBB", "TimeUS,Dec,NumInst,AutoDec,NumEna,LOE,C", "s------", "F------" },  \
    { LOG_RMGI_MSG, RLOG_SIZE(RMGI),                                   \
      "RMGI", "QIffffffBBBB", "TimeUS,LU,OX,OY,OZ,FX,FY,FZ,UFY,H,HSF,I", "s----------#", "F-----------" },                                        \
    { LOG_RBCH_MSG, RLOG_SIZE(RBCH),                                   \
      "RBCH", "QffffiiiBBBB", "TimeUS,PX,PY,PZ,AE,OLat,OLng,OAlt,GVPR,NumInst,ORet,NPtr", "s-----------", "F-----------" },  \
    { LOG_RBCI_MSG, RLOG_SIZE(RBCI),                                   \
      "RBCI", "QIffffBB", "TimeUS,LU,PX,PY,PZ,Dist,H,I", "ssmmmm-#", "F?0000--" }, \
    { LOG_RVOH_MSG, RLOG_SIZE(RVOH),                                   \
      "RVOH", "QfffIBBB", "TimeUS,OX,OY,OZ,Del,H,Ena,NPtr", "s-------", "F-------" },
