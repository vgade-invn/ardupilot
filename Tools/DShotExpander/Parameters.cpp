#include "DShotExpander.h"

/*
 *  DShotExpander parameter definitions
 *
 */

#define GSCALAR(v, name, def) { dshotexpander.g.v.vtype, name, Parameters::k_param_ ## v, &dshotexpander.g.v, {def_value : def} }
#define ASCALAR(v, name, def) { dshotexpander.aparm.v.vtype, name, Parameters::k_param_ ## v, (const void *)&dshotexpander.aparm.v, {def_value : def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &dshotexpander.g.v, {group_info : class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, (const void *)&dshotexpander.v, {group_info : class::var_info} }
#define GOBJECTN(v, pname, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## pname, (const void *)&dshotexpander.v, {group_info : class::var_info} }

const AP_Param::Info DShotExpander::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version,         "FORMAT_VERSION", 0),

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

    // @Group: SERIAL
    // @Path: ../libraries/AP_SerialManager/AP_SerialManager.cpp
    GOBJECT(serial_manager, "SERIAL",   AP_SerialManager),
    
    AP_VAREND
};


void DShotExpander::load_parameters(void)
{
    if (!g.format_version.load() ||
        g.format_version != Parameters::k_format_version) {

        // erase all parameters
        hal.console->printf("Firmware change: erasing EEPROM...\n");
        hal.scheduler->delay_microseconds(100);
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
        hal.console->printf("done.\n");
        hal.scheduler->delay_microseconds(100);
    }

    AP_Param::load_all();
}
