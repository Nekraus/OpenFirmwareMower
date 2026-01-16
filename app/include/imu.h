/*
 * imu.h
 *
 *  Created on: 31/03/2025
 *      Author: Bruno Lecornu
 */

#ifndef IMU_H
#define IMU_H
#include "main.h"
#include <rmw_microros/rmw_microros.h>

/* +-----------------------------------------------------------------------+ */
/* |                        CONSTANTES / MACROS                            | */
/* +-----------------------------------------------------------------------+ */

/* +-----------------------------------------------------------------------+ */
/* |                            TYPEDEFS                                   | */
/* +-----------------------------------------------------------------------+ */
typedef struct
{
  double ax;
  double ay;
  double az;
  double gx;
  double gy;
  double gz;
  float temp;
} imu_data_t;
/* +-----------------------------------------------------------------------+ */
/* |                         GLOBAL VARIABLES                              | */
/* +-----------------------------------------------------------------------+ */

/* +-----------------------------------------------------------------------+ */
/* |                         PUBLIC FUNCTIONS                              | */
/* +-----------------------------------------------------------------------+ */
void IMU_App(ULONG thread_input);
void imu_init_hard(void);
void imu_init_component(void);
void imu_read_data(imu_data_t *imu_data);
void imu_read(uint8_t p_u8Cmd, uint32_t p_u32Size, uint8_t *p_pu8Data);
void imu_write(uint8_t p_u8Cmd, uint32_t p_u32Size, uint8_t *p_pu8Data);
void imu_cmd_readBytes(uint8_t p_u8Cmd, uint32_t p_u32Size, uint8_t *p_pu8Data);
void imu_cmd_writeBytes(uint8_t p_u8Cmd, uint32_t p_u32Size, uint8_t *p_pu8Data);
uint8_t spi2_readwrite(uint8_t byte);

#endif /* IMU_H*/