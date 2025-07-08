/*
 * motors.h
 *
 *  Created on: 25 septembre 2024
 *      Author: Bruno Lecornu
 */

#ifndef MOTORS_H_
#define MOTORS_H_

#include "main.h"
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

/* Init the GPIO and UART to communicate Motors drivers*/ 
void MOTORS_Init(void);
/* App Motors drivers*/
void MOTORS_App(void);

void MOTORS_DMARxIRQ(void);

void MOTORS_setDriveSpeed(int16_t p_s16LeftSpeed, s16 p_s16RightSpeed);
void MOTORS_getDriveSpeed(int16_t *p_ps16LeftSpeed, s16 *p_ps16RightSpeed);
void MOTORS_setMowSpeed(int16_t p_s16MowSpeed);
void MOTORS_getMowSpeed(int16_t *p_ps16MowSpeed);

#endif /* MOTORS_H_ */
