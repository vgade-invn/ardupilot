/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#pragma once

#define USE_POSIX
/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0U
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                24000000U
#endif

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   330U

/*
 * MCU type as defined in the ST header.
 */
#define STM32F427xx

/*
 * APM HW Defines
 */
#define PPM_ICU_TIMER  ICUD4
#define PPM_ICU_CHANNEL  ICU_CHANNEL_2

#define HRT_TIMER GPTD5
#define LINE_LED1 PAL_LINE(GPIOE,12)

#define HAL_STDOUT_SERIAL SD7
#define HAL_STDOUT_BAUDRATE 115200

#include "pins.h"


#ifndef HAL_BOARD_INIT_HOOK_DEFINE
#define HAL_BOARD_INIT_HOOK_DEFINE
#endif
#ifndef HAL_BOARD_INIT_HOOK_CALL
#define HAL_BOARD_INIT_HOOK_CALL
#endif
#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
  HAL_BOARD_INIT_HOOK_DEFINE
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

