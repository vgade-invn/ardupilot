#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Compass/AP_Compass.h>
#include "Parameters.h"
#include "ch.h"

class AP_Periph_FW {
public:
    void init();
    void update();

private:
    void can_start();
    void can_update();

    void load_parameters();

    Parameters g;

    AP_SerialManager serial_manager;
    AP_GPS gps;
    Compass compass;

    // setup the var_info table
    AP_Param param_loader{var_info};

    static const AP_Param::Info var_info[];
};

extern AP_Periph_FW periph;
