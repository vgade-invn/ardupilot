#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_DAL \
    LOG_RFRH_MSG, \
    LOG_RFRF_MSG, \
    LOG_RFRR_MSG, \
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
struct PACKED log_RFRH {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t frame_number;
    float trimx;
    float trimy;
    float trimz;
    float EAS2TAS;
    uint32_t available_memory;
    uint8_t state_bitmask;
    uint8_t rangefinder_ptr_is_null;
    uint8_t get_compass_is_null;
    uint8_t airspeed_ptr_is_null;
    uint8_t fly_forward;
    uint8_t vehicle_class;
    uint8_t ahrs_airspeed_sensor_enabled;
    uint16_t time_flying_ms;
    uint8_t ekf_type;
};

// @LoggerMessage: RFRR
// @Description: Replay vehicle rotation matrix
struct PACKED log_RFRR {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float m0;
    float m1;
    float m2;
    float m3;
    float m4;
    float m5;
    float m6;
    float m7;
    float m8;
};

struct PACKED log_RFRN {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int32_t lat;
    int32_t lng;
    int32_t alt;
};

struct PACKED log_RFRF {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t frame_type;
};

// Replay Data Structure - Inertial Sensor header
struct PACKED log_RISH {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t loop_rate_hz;
    uint8_t primary_gyro;
    uint8_t primary_accel;
    float loop_delta_t;
    uint8_t accel_count;
    uint8_t gyro_count;
    uint32_t last_update_usec;
};
// Replay Data Structure - Inertial Sensor instance data
struct PACKED log_RISI {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    float posx;
    float posy;
    float posz;
    bool use_accel;
    float accelx;
    float accely;
    float accelz;
    bool get_delta_velocity_ret;
    float delta_velocity_x;
    float delta_velocity_y;
    float delta_velocity_z;
    float delta_velocity_dt;
};
// Replay Data Structure - Inertial Sensor instance data
struct PACKED log_RISJ {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    bool use_gyro;
    float gyrox;
    float gyroy;
    float gyroz;
    bool get_delta_angle_ret;
    float delta_angle_x;
    float delta_angle_y;
    float delta_angle_z;
    float delta_angle_dt;
};

// @LoggerMessage: REV2
// @Description: Replay Event
struct PACKED log_REV2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t event;
};

// @LoggerMessage: RSO2
// @Description: Replay Set Origin event
struct PACKED log_RSO2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int32_t lat;
    int32_t lng;
    int32_t alt;
};

// @LoggerMessage: RWA2
// @Description: Replay set-default-airspeed event
struct PACKED log_RWA2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float airspeed;
};

// @LoggerMessage: REV3
// @Description: Replay Event
struct PACKED log_REV3 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t event;
};

// @LoggerMessage: RSO3
// @Description: Replay Set Origin event
struct PACKED log_RSO3 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int32_t lat;
    int32_t lng;
    int32_t alt;
};

// @LoggerMessage: RWA3
// @Description: Replay set-default-airspeed event
struct PACKED log_RWA3 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float airspeed;
};

// @LoggerMessage: REY3
// @Description: Replay Euler Yaw event
struct PACKED log_REY3 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float yawangle;
    float yawangleerr;
    uint32_t timestamp_ms;
    uint8_t type;
};

// @LoggerMessage: RBRH
// @Description: Replay Data Barometer Header
struct PACKED log_RBRH {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t primary;
    uint8_t num_instances;
};

// @LoggerMessage: RBRI
// @Description: Replay Data Barometer Instance
struct PACKED log_RBRI {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    uint32_t last_update_ms;
    bool healthy;
    float altitude;  // from get_altitude
};

// @LoggerMessage: RRNH
// @Description: Replay Data Rangefinder Header
struct PACKED log_RRNH {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    // this is rotation-pitch-270!
    int16_t ground_clearance_cm;
    int16_t max_distance_cm;
};

// @LoggerMessage: RRNI
// @Description: Replay Data Rangefinder Instance
struct PACKED log_RRNI {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    uint8_t orientation;
    uint8_t status;
    uint16_t distance_cm;
    float pos_offset[3];
};

// @LoggerMessage: RGPH
// @Description: Replay Data GPS Header
struct PACKED log_RGPH {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t primary_sensor;
    uint8_t num_sensors;
};

// @LoggerMessage: RGPI
// @Description: Replay Data GPS Instance
struct PACKED log_RGPI {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;

    uint32_t last_message_time_ms;

    uint8_t status;
    int32_t lat;
    int32_t lng;
    int32_t alt;
    bool have_vertical_velocity;
    bool horizontal_accuracy_returncode;
    float hacc;

    bool vertical_accuracy_returncode;
    float vacc;

    uint16_t hdop;

    uint8_t num_sats;

    float lag_sec;
    bool get_lag_returncode;

};

// @LoggerMessage: RGPJ
// @Description: Replay Data GPS Instance - more data
struct PACKED log_RGPJ {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;

    float velocity[3];

    bool speed_accuracy_returncode;
    float sacc;

    float antenna_offset[3];

    // gps-yaw for specific sensor
    bool gps_yaw_deg_returncode;
    float yaw_deg;
    float yaw_accuracy_deg;
};

// Replay Data Structure - Airspeed Sensor header
struct PACKED log_RASH {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t num_sensors;
    uint8_t primary;
};

// Replay Data Structure - Airspeed Sensor instance
struct PACKED log_RASI {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    bool healthy;
    bool use;
    float airspeed;
    uint32_t last_update_ms;
};

// @LoggerMessage: RMGH
// @Description: Replay Data Magnetometer Header
struct PACKED log_RMGH {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float declination;
    uint8_t count;
    bool auto_declination_enabled;
    uint8_t num_enabled;
    bool learn_offsets_enabled;
    bool consistent;
};

// @LoggerMessage: RMGI
// @Description: Replay Data Magnetometer Instance
struct PACKED log_RMGI {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    uint32_t last_update_usec;
    float offsets[3];
    float field[3];
    bool use_for_yaw;
    bool healthy;
    bool have_scale_factor;
};

// @LoggerMessage: RBCH
// @Description: Replay Data Beacon Header
struct PACKED log_RBCH {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t count;
    uint8_t ptr_is_nullptr;
    float vehicle_position_ned[3];
    float accuracy_estimate;
    bool get_vehicle_position_ned_returncode;
    int32_t origin_lat;
    int32_t origin_lng;
    int32_t origin_alt;
    bool get_origin_returncode;
};

// @LoggerMessage: RBCI
// @Description: Replay Data Beacon Instance
struct PACKED log_RBCI {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t last_update_ms;
    float position[3];
    float distance;
    uint8_t healthy;
    uint8_t instance;
};

// @LoggerMessage: RVOH
// @Description: Replay Data Visual Odometry data
struct PACKED log_RVOH {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t healthy;
    uint8_t ptr_is_nullptr;
    float pos_offset[3];
    bool enabled;
    uint32_t delay_ms;
};

#define LOG_STRUCTURE_FROM_DAL        \
    { LOG_RFRH_MSG, sizeof(log_RFRH), \
      "RFRH", "QIffffIBBBBBBBHB", "TimeUS,FN,TX,TY,TZ,E2T,AM,State,NlRF,NlCRP,NlAS,FF,VC,ASE,TF,EKT", "s-------------?-", "F-------------?-" }, \
    { LOG_RFRF_MSG, sizeof(log_RFRF), \
      "RFRF", "QB", "TimeUS,FT", "s-", "F-" }, \
    { LOG_RFRN_MSG, sizeof(log_RFRN), \
      "RFRN", "QIII", "TimeUS,HLat,HLon,HAlt", "sDUm", "FGGB" }, \
    { LOG_RFRR_MSG, sizeof(log_RFRR), \
      "RFRR", "Qfffffffff", "TimeUS,M0,M1,M2,M3,M4,M5,M6,M7,M8", "s---------", "F---------" }, \
    { LOG_REV2_MSG, sizeof(log_REV2), \
      "REV2", "QB", "TimeUS,Event", "s-", "F-" }, \
    { LOG_RSO2_MSG, sizeof(log_RSO2), \
      "RSO2", "QIII", "TimeUS,Lat,Lon,Alt", "sDUm", "FGGB" }, \
    { LOG_RWA2_MSG, sizeof(log_RWA2), \
      "RWA2", "Qf", "TimeUS,Airspeed", "sn", "F0" }, \
    { LOG_REV3_MSG, sizeof(log_REV3), \
      "REV3", "QB", "TimeUS,Event", "s-", "F-" }, \
    { LOG_RSO3_MSG, sizeof(log_RSO3), \
      "RSO3", "QIII", "TimeUS,Lat,Lon,Alt", "sDUm", "FGGB" }, \
    { LOG_RWA3_MSG, sizeof(log_RWA3), \
      "RWA3", "Qf", "TimeUS,Airspeed", "sn", "F0" }, \
    { LOG_REY3_MSG, sizeof(log_REY3), \
      "REY3", "QffIB", "TimeUS,yawangle,yawangleerr,timestamp_ms,type", "s???-", "F???-" }, \
    { LOG_RISH_MSG, sizeof(log_RISH), \
      "RISH", "QHBBfBBI", "TimeUS,LR,PG,PA,LD,AC,GC,LU", "s-------", "F-------" }, \
    { LOG_RISI_MSG, sizeof(log_RISI), \
      "RISI", "QBfffBfffBffff", "TimeUS,I,PX,PY,PZ,UA,AX,AY,AZ,GDVR,DVX,DVY,DVZ,DVDT", "s#------------", "F-------------" }, \
    { LOG_RISJ_MSG, sizeof(log_RISJ), \
      "RISJ", "QBBfffBffff", "TimeUS,I,UG,GX,GY,GZ,GDAR,DAX,DAY,DAZ,DADT", "s#---------", "F----------" }, \
    { LOG_RASH_MSG, sizeof(log_RASH),                           \
      "RASH", "QBB", "TimeUS,Primary,NumInst", "s--", "F--" },  \
    { LOG_RASI_MSG, sizeof(log_RASI),                           \
      "RASI", "QBBBfI", "TimeUS,I,H,Use,Spd,UpdateMS", "s#?-??", "F-----" }, \
    { LOG_RBRH_MSG, sizeof(log_RBRH),                           \
      "RBRH", "QBB", "TimeUS,Primary,NumInst", "s--", "F--" },  \
    { LOG_RBRI_MSG, sizeof(log_RBRI),                           \
      "RBRI", "QBIBf", "TimeUS,I,LastUpdate,H,Alt", "s#---", "F----" }, \
    { LOG_RRNH_MSG, sizeof(log_RRNH),                           \
      "RRNH", "Qhh", "TimeUS,GCl,MaxD", "s??", "F??" },  \
    { LOG_RRNI_MSG, sizeof(log_RRNI),                           \
      "RRNI", "QBBBHfff", "TimeUS,I,Orient,Status,Dist,OX,OY,OZ", "s#--????", "F---????" }, \
    { LOG_RGPH_MSG, sizeof(log_RGPH),                           \
      "RGPH", "QBB", "TimeUS,Primary,NumInst", "s--", "F--" },  \
    { LOG_RGPI_MSG, sizeof(log_RGPI),                           \
      "RGPI", "QBIBiiiBBfBfHBfB", "TimeUS,I,LMT,Stat,lat,lon,alt,hvv,harc,ha,varc,va,hdp,ns,lgrc,lg", "s#--------------", "F---------------" }, \
    { LOG_RGPJ_MSG, sizeof(log_RGPJ),                           \
      "RGPJ", "QBfffBffffBff", "TimeUS,I,vx,vy,vz,sarc,sa,aox,aoy,aoz,ydrc,yd,yda", "s#-----------", "F------------" }, \
    { LOG_RMGH_MSG, sizeof(log_RMGH),                           \
      "RMGH", "QBBfBBB", "TimeUS,Dec,NumInst,AutoDec,NumEna,LOE,C", "s------", "F------" },  \
    { LOG_RMGI_MSG, sizeof(log_RMGI),                           \
      "RMGI", "QBIffffffBBB", "TimeUS,I,LU,OX,OY,OZ,FX,FY,FZ,UFY,H,HSF", "s#----------", "F-----------" },                                        \
    { LOG_RBCH_MSG, sizeof(log_RBCH),                           \
      "RBCH", "QBBffffBfffB", "TimeUS,NumInst,NPtr,PX,PY,PZ,AE,GVPR,OLat,OLng,OAlt,ORet", "s#-mmmm-DUm-", "F--0000-GGB-" },  \
    { LOG_RBCI_MSG, sizeof(log_RBCI),                           \
      "RBCI", "QIffffBB", "TimeUS,LU,PX,PY,PZ,Dist,H,I", "ssmmmm-#", "F?0000--" }, \
    { LOG_RVOH_MSG, sizeof(log_RVOH),                           \
      "RVOH", "QBBfffBI", "TimeUS,H,NPtr,OX,OY,OZ,Ena,Del", "s--mmm-s", "F--000-?" },
