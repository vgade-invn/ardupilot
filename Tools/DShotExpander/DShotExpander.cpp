/*
  DShot expansion firmware
*/

#include "DShotExpander.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(DShotExpander, &dshotexpander, func, rate_hz, max_time_micros)

const AP_Scheduler::Task DShotExpander::scheduler_tasks[] = {
    SCHED_TASK(fast_loop,  1000, 100),
    SCHED_TASK(one_hz_loop,  1, 100),
    SCHED_TASK(gcs_check_input,      400,    180),
    SCHED_TASK(gcs_send_heartbeat,     1,    110),
    SCHED_TASK(gcs_send_deferred,     50,    550),
    SCHED_TASK(gcs_data_stream_send,  50,    550),
};


void DShotExpander::setup()
{
    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    hal.scheduler->delay(3000);
    
    load_parameters();

    serial_manager.init();
    gcs().chan(0).setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 0);
    gcs().setup_uarts(serial_manager);

    BoardConfig.init();
    BoardConfig.init_safety();

    hal.rcout->set_esc_scaling(1000, 2000);

    hal.rcout->set_freq(0xFF, 400);
    hal.rcout->set_output_mode(0xFF, AP_HAL::RCOutput::MODE_PWM_DSHOT150);

    setup_uart();
    
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE_SIMPLE(scheduler_tasks), 0);
}


void DShotExpander::one_hz_loop()
{
    SRV_Channels::enable_aux_servos();
    hal.uartB->printf("tick\n");
}


void DShotExpander::loop()
{
    scheduler.loop();
    hal.scheduler->delay_microseconds(20000);
}

/*
  constructor for main DShotExpander class
 */
DShotExpander::DShotExpander(void)
    : param_loader(var_info),
      DataFlash(fwver.fw_string, g.log_bitmask)
{
}

void DShotExpander::fast_loop(void)
{
}

DShotExpander dshotexpander;

AP_HAL_MAIN_CALLBACKS(&dshotexpander);
