/*
  DShot expansion firmware
*/

#include "DShotExpander.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void DShotExpander::setup() {
}

void DShotExpander::loop()
{
    hal.console->printf("tick\n");
    hal.scheduler->delay(1000);
}

/*
  constructor for main DShotExpander class
 */
DShotExpander::DShotExpander(void)
{
}

DShotExpander dshotexpander;

AP_HAL_MAIN_CALLBACKS(&dshotexpander);
