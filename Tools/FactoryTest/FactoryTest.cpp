/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  FacttoryTest firmware
  Andrew Tridgell September 2016
*/
/*
  This firmware is intended for use in initial testing of boards in a
  factory setting. It tests basic operation of the microcontrollers
  and primary sensors. This should be used in conjunction with the
  full test system which does a full 3D calibration and sensor test
  after assembly
 */

#include <cmath>
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <AP_Common/AP_Common.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Math/AP_Math.h>
#include <AP_ADC/AP_ADC.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AccelCal/AP_AccelCal.h>
#include <AP_AHRS/AP_AHRS.h>
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#define NUM_ACCEL 3
#define NUM_GYRO 3
#define NUM_BARO 1
#define NUM_MAG 2


static AP_BoardConfig BoardConfig;
static AP_GPS gps;
static AP_Baro baro;
static Compass compass;
static AP_InertialSensor ins;
static AP_SerialManager serial_manager;


const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static void printmsg(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    hal.console->vprintf(fmt, ap);
    vprintf(fmt, ap);
    va_end(ap);
}

/*
  initialise sensors
 */
void setup();
void setup()
{
    printmsg("Starting setup\n");
    serial_manager.init_console();
    serial_manager.init();
    BoardConfig.init();
    baro.init();
    compass.init();
    ins.init(50);
    baro.calibrate(); 
    printmsg("done setup\n");
}

/*
  test FRAM
 */
static bool test_fram(void)
{
    // we skip writing offset 8188 as that is used on HAL_PX4 for the
    // MTD signature
    const uint32_t mtd_sig_ofs = 8192-4;
    for (uint32_t ofs=0; ofs<HAL_STORAGE_SIZE; ofs += 4) {
        if (ofs == mtd_sig_ofs) {
            continue;
        }
        uint32_t v = ofs;
        hal.storage->write_block(ofs, &v, 4);
    }
    if (!hal.storage->sync()) {
        printmsg("FRAM sync failed\n");
        return false;
    }
    if (!hal.storage->reopen()) {
        printmsg("FRAM reopen failed\n");
        return false;
    }
    for (uint32_t ofs=0; ofs<HAL_STORAGE_SIZE; ofs += 4) {
        if (ofs == mtd_sig_ofs) {
            continue;
        }
        uint32_t v = 0;
        hal.storage->read_block(&v, ofs, 4);
        if (v != ofs) {
            printmsg("Bad FRAM data %u at offset %u\n", (unsigned)v, (unsigned)ofs);
            return false;
        }
    }
    return true;
}

/*
  test accelerometers
 */
static bool test_accel(void)
{
    bool ret = true;

    ins.wait_for_sample();
    ins.update();
    
    for (uint8_t i=0; i<NUM_ACCEL; i++) {
        if (!ins.get_accel_health(i)) {
            printmsg("accel %u not healthy\n", (unsigned)i);
            ret = false;
            continue;
        }
        const Vector3f &a = ins.get_accel(i);
        if (a.length() < 7 || a.length() > 12) {
            printmsg("accel %u invalid length %.1f\n", (unsigned)i, a.length());
            ret = false;
            continue;
        }
    }
    return ret;
}


/*
  test gyros
 */
static bool test_gyro(void)
{
    bool ret = true;

    ins.wait_for_sample();
    ins.update();
    
    for (uint8_t i=0; i<NUM_GYRO; i++) {
        if (!ins.get_gyro_health(i)) {
            printmsg("accel %u not healthy\n", (unsigned)i);
            ret = false;
            continue;
        }
        const Vector3f &g = ins.get_gyro(i);
        if (g.length() <= 0 || g.length() > 5) {
            printmsg("gyro %u invalid length %.1f\n", (unsigned)i, g.length());
            ret = false;
            continue;
        }
    }
    return ret;
}


/*
  test baros
 */
static bool test_baro(void)
{
    bool ret = true;

    baro.update();
    
    for (uint8_t i=0; i<NUM_BARO; i++) {
        if (!baro.healthy(i)) {
            printmsg("baro %u not healthy\n", (unsigned)i);
            ret = false;
            continue;
        }
        if (baro.get_pressure() < 80000.0f || baro.get_pressure() > 180000.0f) {
            printmsg("baro %u invalid pressure %.1f\n", (unsigned)i, baro.get_pressure());
            ret = false;
            continue;
        }
        if (baro.get_temperature() < 10 || baro.get_temperature() > 70) {
            printmsg("baro %u invalid temperature %.1f\n", (unsigned)i, baro.get_temperature());
            ret = false;
            continue;            
        }
    }
    return ret;
}


/*
  test compasses
 */
static bool test_compass(void)
{
    bool ret = true;

    for (uint8_t i=0; i<NUM_MAG; i++) {
        if (!compass.read() || !compass.healthy(i)) {
            printmsg("mag %u not healthy\n", (unsigned)i);
            ret = false;
            continue;
        }
        const Vector3f &field = compass.get_field(i);
        if (field.length() < 100 || field.length() > 2000) {
            printmsg("mag %u invalid field length %.1f\n", (unsigned)i, field.length());
            ret = false;
            continue;
        }
    }
    return ret;
}

static struct {
    const char *name;
    bool (*test)(void);
} tests[] = {
    { "FRAM", test_fram },
    { "Accel", test_accel },
    { "Gyro", test_gyro },
    { "Baro", test_baro },
    { "Compass", test_compass },
};

void loop()
{
    uint8_t failures = 0;
    for (uint8_t i=0; i<ARRAY_SIZE(tests); i++) {
        printmsg("**** Running test %s\n", tests[i].name);
        if (!tests[i].test()) {
            failures++;
            printmsg("##### FAILED TEST %s #####\n", tests[i].name);
        }
    }
    if (failures == 0) {
		printmsg("\n");
		printmsg(" /\\  __ \\   /\\ \\       /\\ \\          /\\  __ \\   /\\ \\/ /    \n");
		printmsg(" \\ \\  __ \\  \\ \\ \\____  \\ \\ \\____     \\ \\ \\/\\ \\  \\ \\  _\"-.  \n");
		printmsg("  \\ \\_\\ \\_\\  \\ \\_____\\  \\ \\_____\\     \\ \\_____\\  \\ \\_\\ \\_\\ \n");
		printmsg("   \\/_/\\/_/   \\/_____/   \\/_____/      \\/_____/   \\/_/\\/_/ \n");
    } else {
		printmsg("\n");
		printmsg(" /\\  ___\\ /\\  __ \\   /\\ \\   /\\ \\    \n");
		printmsg(" \\ \\  __\\ \\ \\  __ \\  \\ \\ \\  \\ \\ \\__\n");
		printmsg("  \\ \\_\\    \\ \\_\\ \\_\\  \\ \\_\\  \\ \\_____\\ \n");
		printmsg("   \\/_/     \\/_/\\/_/   \\/_/   \\/_____/ \n");
		printmsg("\n");
    }
    while (true) {
        hal.scheduler->delay(1000);
    }
}

AP_HAL_MAIN();
