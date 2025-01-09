/*
 * imu.c 
 *
 *  Created on: 18/12/2024 
 *      Author: Bruno Lecornu 
 */

/* +-----------------------------------------------------------------------+ */
/* |                               HEADER                                  | */
/* +-----------------------------------------------------------------------+ */
#include "main.h"
/* +-----------------------------------------------------------------------+ */
/* |                            TYPEDEFS                                   | */
/* +-----------------------------------------------------------------------+ */

/* +-----------------------------------------------------------------------+ */
/* |                        CONSTANTES / MACROS                            | */
/* +-----------------------------------------------------------------------+ */

/* +-----------------------------------------------------------------------+ */
/* |                         GLOBAL VARIABLES                              | */
/* +-----------------------------------------------------------------------+ */

/* +-----------------------------------------------------------------------+ */
/* |                         LOCAL VARIABLES                               | */
/* +-----------------------------------------------------------------------+ */

/* +-----------------------------------------------------------------------+ */
/* |                         Prototype FUNCTIONS                           | */
/* +-----------------------------------------------------------------------+ */

/* +-----------------------------------------------------------------------+ */
/* |                         PUBLIC FUNCTIONS                              | */
/* +-----------------------------------------------------------------------+ */
void IMU_Init(void){
    spi_parameter_struct spi_init_struct;

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOF);
    rcu_periph_clock_enable(RCU_SPI2);

    gpio_init(GPIOA, GPIO_MODE_AF_OD, GPIO_OSPEED_MAX, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
    gpio_init(GPIOF, GPIO_MODE_AF_OD, GPIO_OSPEED_MAX, GPIO_PIN_15);

    /* deinitilize SPI and the parameters */
    spi_i2s_deinit(SPI2);
    spi_struct_para_init(&spi_init_struct);

    /* configure SPI2 parameter */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT ;
    spi_init_struct.prescale             = SPI_PSC_64;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI2, &spi_init_struct);
    spi_enable(SPI2);
}
/* +-----------------------------------------------------------------------+ */
/* |                           LOCAL FUNCTIONS                             | */
/* +-----------------------------------------------------------------------+ */

void imu_read(uint8_t p_u8Cmd, uint32_t p_u32Size, uint8_t *p_pu8Data){
    uint8_t tmp = (0x7f < p_u8Cmd);
    /* set the current bank address*/
    imu_cmd_writeBytes(0x7f,&tmp,1);
    /* read the data*/
    imu_cmd_readBytes(p_u8Cmd|0x80,p_u32Size,p_pu8Data);
}

void imu_write(uint8_t p_u8Cmd, uint32_t p_u32Size, uint8_t *p_pu8Data){
    uint8_t tmp = (0x7f < p_u8Cmd);
    /* set the current bank address*/
    imu_cmd_writeBytes(0x7f,&tmp,1);
    /* write the data*/
    imu_cmd_writeBytes(p_u8Cmd|0x80,p_u32Size,p_pu8Data);
}

void imu_cmd_readBytes(uint8_t p_u8Cmd, uint32_t p_u32Size, uint8_t *p_pu8Data){
    gpio_bit_reset(GPIOF, GPIO_PIN_15);
    spi_i2s_data_transmit(SPI2,p_u8Cmd);
    for (uint32_t i = 0; i < p_u32Size; i ++) {
        *(p_pu8Data + i) = spi_i2s_data_receive(SPI2);
    }
    gpio_bit_set(GPIOF, GPIO_PIN_15);
}

void imu_cmd_writeBytes(uint8_t p_u8Cmd, uint32_t p_u32Size, uint8_t *p_pu8Data){
    gpio_bit_reset(GPIOF, GPIO_PIN_15);
    spi_i2s_data_transmit(SPI2,p_u8Cmd);
    for (uint32_t i = 0; i < p_u32Size; i ++) {
        spi_i2s_data_transmit(SPI2,*(p_pu8Data + i));
    }
    gpio_bit_set(GPIOF, GPIO_PIN_15);
}