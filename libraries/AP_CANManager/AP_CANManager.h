#pragma once

#include <AP_HAL/AP_HAL.h>

#if MAX_NUMBER_OF_CAN_INTERFACES

#include <AP_Param/AP_Param.h>
#include "AP_CANProtocol.h"
#include "AP_SLCANDriver.h"
class AP_CANManager {
public:
    AP_CANManager();

    /* Do not allow copies */
    AP_CANManager(const AP_CANManager &other) = delete;
    AP_CANManager &operator=(const AP_CANManager&) = delete;

    static AP_CANManager* get_singleton() {
        return _singleton;
    }

    enum Protocol_Type : uint8_t {
        Protocol_Type_None = 0,
        Protocol_Type_UAVCAN = 1,
        Protocol_Type_KDECAN = 2,
        Protocol_Type_ToshibaCAN = 3,
        Protocol_Type_PiccoloCAN = 4,
    };

    void init(void);

    // returns number of active CAN Protocols
    uint8_t get_num_drivers(void) const { return _num_protocols; }

    uint8_t get_debug_level_driver(uint8_t interface) { return 2; }

    // return driver for index i
    AP::CANProtocol* get_driver(uint8_t i) const {
        if (i < MAX_NUMBER_OF_CAN_INTERFACES) {
            return _protocols[i]._protocol;
        }
        return nullptr;
    }

    // return protocol type index i
    Protocol_Type get_protocol_type(uint8_t i) const {
        if (i < MAX_NUMBER_OF_CAN_INTERFACES) {
            return _protocols[i]._protocol_type_cache;
        }
        return Protocol_Type_None;
    }

    static const struct AP_Param::GroupInfo var_info[];

private:
    class Interface {
        friend class AP_CANManager;

    public:
        Interface() {
            AP_Param::setup_object_defaults(this, var_info);
        }

        static const struct AP_Param::GroupInfo var_info[];

    private:
        AP_Int8 _protocol_number;
        uint8_t _protocol_number_cache;
        AP_Int32 _bitrate;
    };

    class Protocol {
        friend class AP_CANManager;

    public:
        Protocol() {
            AP_Param::setup_object_defaults(this, var_info);
        }

        static const struct AP_Param::GroupInfo var_info[];

    private:
        AP_Int8 _protocol_type;
        Protocol_Type _protocol_type_cache;
        AP::CANProtocol* _protocol;
        AP::CANProtocol* _uavcan;   // UAVCAN
        AP::CANProtocol* _kdecan;   // KDECAN
        AP::CANProtocol* _tcan;     // ToshibaCAN
        AP::CANProtocol* _pcan;     // PiccoloCAN
    };

    Interface _interfaces[MAX_NUMBER_OF_CAN_INTERFACES];
    Protocol _protocols[MAX_NUMBER_OF_CAN_PROTOCOLS];
    AP_Int8 _slcan_can_port;
    AP_Int8 _slcan_ser_port;
    AP_Int8 _slcan_timeout;
    AP_Int8 _slcan_mode;
    uint8_t _num_protocols;
    SLCAN::CANDriver slcan_driver;
    static AP_CANManager *_singleton;

    void slcan_passthrough_loop();
};

namespace AP {
    AP_CANManager& can();
}

#endif
