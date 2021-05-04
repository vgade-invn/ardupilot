#pragma once

/*
  allow for builds with either single or double precision EKF
 */

#include <AP_HAL/AP_HAL.h>

#if HAL_EKF_DOUBLE
typedef double ftype;
#else
typedef float ftype;
#endif
