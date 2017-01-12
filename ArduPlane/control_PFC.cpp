/*
  simple framework for control research for MCovinus
  See discussions on https://gitter.im/ArduPilot/Research
 */


#include "Plane.h"

/*
  called from Plane::servos_output(), after main mixing but before PWM
  calculation
 */
void Plane::PFC_servos_hook(void)
{
    if (g2.pfc_chan <= 0 || hal.rcin->read(g2.pfc_chan-1) < 1700) {
        // not enabled
        return;
    }

    // PFC is enabled, lets change the flaps


    // get airspeed
    float aspeed;
    if (!ahrs.airspeed_estimate(&aspeed)) {
        // we don't have an airspeed estimate, bail
    }

    if (aspeed < 5) {
        // we're not moving fast enough, bail
        return;
    }
    
    // set flaps based on airspeed, silly example calculation
    float flaps = 30 / aspeed;

    // constrain to 0 to 100%
    flaps = constrain_float(flaps, 0, 100);

    // set flaps
    SRV_Channels::set_output_scaled(SRV_Channel::k_flap, flaps);

    // lets log what we're doing too
    DataFlash_Class::instance()->Log_Write("PFC", "TimeUS,Aspd,Flaps", "Qff",
                                           AP_HAL::micros64(),
                                           aspeed,
                                           flaps);
}
