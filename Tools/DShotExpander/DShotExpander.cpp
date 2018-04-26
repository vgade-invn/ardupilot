/*
  DShot expansion firmware
*/

#include "DShotExpander.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(DShotExpander, &dshotexpander, func, rate_hz, max_time_micros)

const AP_Scheduler::Task DShotExpander::scheduler_tasks[] = {
    SCHED_TASK(fast_loop,  1000, 100),
    SCHED_TASK(one_hz_loop,  1, 100),
    SCHED_TASK(gcs_send_heartbeat,  1,  100),
//    SCHED_TASK(gcs_send_deferred,     50,    550),
//    SCHED_TASK(gcs_data_stream_send,  50,    550),
};


void DShotExpander::setup()
{
    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    hal.scheduler->delay(3000);
    
    hal.console->printf("setup\n");
    // initialise the main loop scheduler

    load_parameters();

    serial_manager.init();
    gcs().chan(0).setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 0);
    gcs().setup_uarts(serial_manager);
    
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE_SIMPLE(scheduler_tasks), 0);
}


void DShotExpander::one_hz_loop()
{
    hal.console->printf("tick\n");
}


void DShotExpander::loop()
{
    scheduler.loop();
    hal.scheduler->delay_microseconds(2500);
}

/*
  constructor for main DShotExpander class
 */
DShotExpander::DShotExpander(void)
{
}

void DShotExpander::fast_loop(void)
{
    hal.scheduler->delay_microseconds(500);    
}

DShotExpander dshotexpander;

AP_HAL_MAIN_CALLBACKS(&dshotexpander);
