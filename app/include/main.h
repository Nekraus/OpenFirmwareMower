/*
 * main.h
 *
 *  Created on: 25 septembre 2024
 *      Author: Bruno Lecornu
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdio.h>
#include <stdbool.h>

#include "tx_api.h"

#include "gd32f30x.h"
#include "gd32f30x_it.h"
#include "cmsis_utils.h"

#include "../include/main.h"
#include "../include/digitalIO.h"
#include "../include/display.h"
#include "../include/eeprom.h"
#include "../include/imu.h"
#include "../include/motors.h"
#include "../include/battery.h"
#include "../include/adc.h"

#include "../include/microros_transports.h"

#include <errno.h>
#include <sys/unistd.h>

/* +-----------------------------------------------------------------------+ */
/* |                        CONSTANTES / MACROS                            | */
/* +-----------------------------------------------------------------------+ */

/* +-----------------------------------------------------------------------+ */
/* |                            TYPEDEFS                                   | */
/* +-----------------------------------------------------------------------+ */

/* +-----------------------------------------------------------------------+ */
/* |                         GLOBAL VARIABLES                              | */
/* +-----------------------------------------------------------------------+ */

/* +-----------------------------------------------------------------------+ */
/* |                         PUBLIC FUNCTIONS                              | */
/* +-----------------------------------------------------------------------+ */

void thread_sleepUntil(uint32_t *const previousWakeTime, const uint32_t timeIncrement);

#endif /* MAIN_H_ */