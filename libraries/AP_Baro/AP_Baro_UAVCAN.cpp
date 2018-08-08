#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include "AP_Baro_UAVCAN.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#if HAL_OS_POSIX_IO
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#endif

#include <uavcan/equipment/air_data/StaticPressure.hpp>
#include <uavcan/equipment/air_data/StaticTemperature.hpp>

extern const AP_HAL::HAL& hal;

#define debug_baro_uavcan(level_debug, can_driver, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(can_driver)) { printf(fmt, ##args); }} while (0)

//UAVCAN Frontend Registry Binder
UC_REGISTRY_BINDER(PressureCb, uavcan::equipment::air_data::StaticPressure);
UC_REGISTRY_BINDER(TemperatureCb, uavcan::equipment::air_data::StaticTemperature);

AP_Baro_UAVCAN::DetectedModules AP_Baro_UAVCAN::_detected_modules[] = {0};
AP_HAL::Semaphore* AP_Baro_UAVCAN::_sem_registry = nullptr;

void AP_Baro_UAVCAN::subscribe_baro_uavcan_messages(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr) {
        return;
    }

    auto* node = ap_uavcan->get_node();

    uavcan::Subscriber<uavcan::equipment::air_data::StaticPressure, PressureCb> *pressure_listener;
    pressure_listener = new uavcan::Subscriber<uavcan::equipment::air_data::StaticPressure, PressureCb>(*node);
    const int pressure_listener_res = pressure_listener->start(PressureCb(ap_uavcan, &handle_pressure));                     //Msg Handler
    if (pressure_listener_res < 0) {
        AP_HAL::panic("UAVCAN Baro subscriber start problem\n\r");
        return;
    }

    uavcan::Subscriber<uavcan::equipment::air_data::StaticTemperature, TemperatureCb> *temperature_listener;
    temperature_listener = new uavcan::Subscriber<uavcan::equipment::air_data::StaticTemperature, TemperatureCb>(*node);
    const int temperature_listener_res = temperature_listener->start(TemperatureCb(ap_uavcan, &handle_temperature));                          //Msg Handler
    if (temperature_listener_res < 0) {
        AP_HAL::panic("UAVCAN Baro subscriber start problem\n\r");
        return;
    }
}

void AP_Baro_UAVCAN::handle_pressure(AP_UAVCAN* ap_uavcan, uint8_t node_id, const PressureCb &cb)
{
    AP_Baro_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, true);
    if (driver == nullptr) {
        return;
    }
    if (driver->_sem_baro->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {        
        driver->_pressure = cb.msg->static_pressure;
        driver->new_pressure = true;
        driver->_sem_baro->give();
    }
}

void AP_Baro_UAVCAN::handle_temperature(AP_UAVCAN* ap_uavcan, uint8_t node_id, const TemperatureCb &cb)
{
    AP_Baro_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, false);
    if (driver == nullptr) {
        return;
    }
    if (driver->_sem_baro->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        driver->_temperature = cb.msg->static_temperature;
        driver->_sem_baro->give();
    }
}



AP_Baro_UAVCAN* AP_Baro_UAVCAN::get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, bool create_new)
{
    AP_Baro_UAVCAN* driver = nullptr;
    if (ap_uavcan == nullptr) {
        return nullptr;
    }
    AP_Baro& _frontend = AP::baro();
    for (uint8_t i = 0; i < _frontend._num_drivers; i++) {
        if (_frontend.drivers[i] == nullptr) {
            continue;
        }
        if (_frontend.drivers[i]->get_uavcan_manager() == ap_uavcan && _frontend.drivers[i]->get_uavcan_node() == node_id) {
            return (AP_Baro_UAVCAN*)_frontend.drivers[i];
        }
    }
    
    if (driver == nullptr && create_new) {
        if (take_registry()) {
            bool already_detected = false;
            //Check if there's an empty spot for possible registeration
            for (uint8_t i = 0; i < BARO_MAX_DRIVERS; i++) {
                if (_detected_modules[i].ap_uavcan == ap_uavcan && _detected_modules[i].node_id == node_id) {
                    //Already Detected
                    already_detected = true;
                }
            }
            if (!already_detected) {
                for (uint8_t i = 0; i < BARO_MAX_DRIVERS; i++) {
                    if (_detected_modules[i].ap_uavcan == nullptr) {
                        _detected_modules[i].ap_uavcan = ap_uavcan;
                        _detected_modules[i].node_id = node_id;
                        break;
                    }
                }
            }
            give_registry();
        }
    }
    return driver;
}

bool AP_Baro_UAVCAN::take_registry()
{
    if (_sem_registry == nullptr) {
        _sem_registry = hal.util->new_semaphore();
    }
    return _sem_registry->take(HAL_SEMAPHORE_BLOCK_FOREVER);
}

void AP_Baro_UAVCAN::give_registry()
{
    _sem_registry->give();
}

AP_Baro_Backend* AP_Baro_UAVCAN::probe(AP_Baro &baro)
{
    if (!take_registry()) {
        return nullptr;
    }
    AP_Baro_UAVCAN* backend = nullptr;
    for (uint8_t i = 0; i < BARO_MAX_DRIVERS; i++) {
        if (_detected_modules[i].ap_uavcan != nullptr) {
            backend = new AP_Baro_UAVCAN(baro);
            if (backend == nullptr) {
                debug_baro_uavcan(2, _detected_modules[i].ap_uavcan->get_driver_num(),"Failed register UAVCAN Baro Node %d on Bus %d\n", _detected_modules[i].node_id, _detected_modules[i].ap_uavcan->get_driver_num());
            } else {
                backend->_ap_uavcan = _detected_modules[i].ap_uavcan;
                backend->_node_id = _detected_modules[i].node_id;
                backend->register_sensor();
                debug_baro_uavcan(2, _detected_modules[i].ap_uavcan->get_driver_num(),"Registered UAVCAN Baro Node %d on Bus %d\n", _detected_modules[i].node_id, _detected_modules[i].ap_uavcan->get_driver_num());
            }
            _detected_modules[i].ap_uavcan = nullptr;
            break;
        }
    }
    give_registry();
    return backend;
}

/*
  constructor - registers instance at top Baro driver
 */
AP_Baro_UAVCAN::AP_Baro_UAVCAN(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
    _sem_baro = hal.util->new_semaphore();
}

AP_Baro_UAVCAN::~AP_Baro_UAVCAN()
{
    delete _sem_baro;
}

// Read the sensor
void AP_Baro_UAVCAN::update(void)
{
    if (new_pressure && _sem_baro->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        _copy_to_frontend(_instance, _pressure, _temperature);

        _frontend.set_external_temperature(_temperature);
        new_pressure = false;
        _sem_baro->give();
    }
}

#endif // HAL_WITH_UAVCAN

