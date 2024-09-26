/*
 * motors.c
 *
 *  Created on: 25 septembre 2024
 *      Author: Bruno Lecornu
 */

/* +-----------------------------------------------------------------------+ */
/* |                               HEADER                                  | */
/* +-----------------------------------------------------------------------+ */
#include "motors.h"

/* +-----------------------------------------------------------------------+ */
/* |                            TYPEDEFS                                   | */
/* +-----------------------------------------------------------------------+ */
typedef enum {
    MOTORS_SELECT_RIGHT = 0,
    MOTORS_SELECT_LEFT = 1,
    MOTORS_SELECT_MOWER = 3,
    MOTORS_SELECT_MAX
} MOTORS_SELECT_e;

typedef struct {
uint16_t u16Header;
uint8_t u8Motor_id;
int16_t s16Speed;
uint16_t u16Voltage;

} motor_speed_tx_t;
/* +-----------------------------------------------------------------------+ */
/* |                        CONSTANTES / MACROS                            | */
/* +-----------------------------------------------------------------------+ */
#define TX_SIZE 11
#define RX_SIZE 11


/* +-----------------------------------------------------------------------+ */
/* |                         GLOBAL VARIABLES                              | */
/* +-----------------------------------------------------------------------+ */

/* +-----------------------------------------------------------------------+ */
/* |                         LOCAL VARIABLES                               | */
/* +-----------------------------------------------------------------------+ */

uint8_t txbuffer[TX_SIZE];
uint8_t rxbuffer[RX_SIZE];
/* +-----------------------------------------------------------------------+ */
/* |                         Prototype FUNCTIONS                              | */
/* +-----------------------------------------------------------------------+ */
void motors_selectMotor(MOTORS_SELECT_e p_eSelectedMotor);
void motors_prepareInit_1(MOTORS_SELECT_e p_eSelectedMotor, uint8_t* p_data);
void motors_prepareInit_2(MOTORS_SELECT_e p_eSelectedMotor, uint8_t* p_data);
void motors_prepareInit_3(MOTORS_SELECT_e p_eSelectedMotor, uint8_t* p_data);

/* +-----------------------------------------------------------------------+ */
/* |                         PUBLIC FUNCTIONS                              | */
/* +-----------------------------------------------------------------------+ */
/*!
    \brief   MOTORS_Init();
    Init all the periph for the motors use
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MOTORS_Init(void)
{
    dma_parameter_struct dma_init_struct;
    /* enable DMA0 */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_DMA0);
    rcu_periph_clock_enable(RCU_USART1);

    nvic_irq_enable(DMA0_Channel5_IRQn, 0, 0);
    nvic_irq_enable(DMA0_Channel6_IRQn, 0, 1);
    nvic_irq_enable(USART1_IRQn, 0, 1);

    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_3);

    usart_deinit(USART1);
    usart_baudrate_set(USART1, 115200U);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
    usart_interrupt_enable(USART1, USART_INT_IDLE);
    usart_enable(USART1);
    
    /* deinitialize DMA channel6(USART1 tx) */
    dma_deinit(DMA0, DMA_CH6);
    dma_struct_para_init(&dma_init_struct);

    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr = (uint32_t)txbuffer;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = TX_SIZE;
    dma_init_struct.periph_addr = (uint32_t)&USART_DATA(USART1);
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA0, DMA_CH6, &dma_init_struct);

    /* deinitialize DMA channel5 (USART1 rx) */
    dma_deinit(DMA0, DMA_CH5);
    dma_struct_para_init(&dma_init_struct);

    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr = (uint32_t)rxbuffer;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = RX_SIZE;
    dma_init_struct.periph_addr = (uint32_t)&USART_DATA(USART1);
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA0, DMA_CH5, &dma_init_struct);

    /* configure DMA mode */
    dma_circulation_disable(DMA0, DMA_CH6);
    dma_memory_to_memory_disable(DMA0, DMA_CH6);
    dma_circulation_disable(DMA0, DMA_CH5);
    dma_memory_to_memory_disable(DMA0, DMA_CH5);
    
    /* enable USART DMA for reception */
    usart_dma_receive_config(USART1, USART_DENT_ENABLE);
    /* enable DMA0 channel4 transfer complete interrupt */
    dma_interrupt_enable(DMA0, DMA_CH5, DMA_INT_FTF);
    /* enable DMA0 channel4 */
    dma_channel_enable(DMA0, DMA_CH5);
    /* enable USART DMA for transmission */
    usart_dma_transmit_config(USART1, USART_DENT_ENABLE);
    /* enable DMA0 channel3 transfer complete interrupt */
    dma_interrupt_enable(DMA0, DMA_CH6, DMA_INT_FTF);
    /* enable DMA0 channel3 */
    dma_channel_enable(DMA0, DMA_CH6);

    /* Mux Control */
    gpio_init(GPIOE, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
    gpio_init(GPIOE, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
}

/* +-----------------------------------------------------------------------+ */
/* |                           LOCAL FUNCTIONS                             | */
/* +-----------------------------------------------------------------------+ */

/*!
    \brief   motors_selectMotor(MOTORS_SELECT_e p_eSelectedMotor)
    Set the mux for the selected motor
    \param[in]  MOTORS_SELECT_e p_eSelectedMotor
    \param[out] none
    \retval     none
*/
void motors_selectMotor(MOTORS_SELECT_e p_eSelectedMotor){
    if(p_eSelectedMotor < MOTORS_SELECT_MAX){
        gpio_bit_write(GPIOE,GPIO_PIN_7,p_eSelectedMotor&0x01);
        gpio_bit_write(GPIOE,GPIO_PIN_8,p_eSelectedMotor&0x02);
        //delay_1ms(1);
    }
}

/*!
    \brief   void motors_Calculatechecksum(uint8_t* p_pu8Buffer)
    Calculate the CRC for UART communication 
    \param[in]  uint8_t* p_pu8Buffer buffer to send or received 
    \param[out] none
    \retval     u8 CRC
*/
void motors_Calculatechecksum(uint8_t* p_pu8Buffer){
    if(p_pu8Buffer == NULL){
        return 0;
    }
    else{
        return ((p_pu8Buffer[2] + p_pu8Buffer[3] + p_pu8Buffer[5] + p_pu8Buffer[6] + \
                p_pu8Buffer[7] + p_pu8Buffer[8] + p_pu8Buffer[9] + p_pu8Buffer[10]   \
        ) & 0x7f );
    }

}

void motors_prepareInit_1(MOTORS_SELECT_e p_eSelectedMotor, uint8_t* p_data){
    p_data[0] = p_eSelectedMotor & 0x03;
    p_data[1] = 0;
    p_data[2] = 0;
    p_data[3] = 0x04;
    p_data[4] = 0;
    p_data[5] = 0;
    p_data[6] = 0;
    p_data[7] = 0;
}
void motors_prepareInit_2(MOTORS_SELECT_e p_eSelectedMotor, uint8_t* p_data){
    p_data[0] = p_eSelectedMotor & 0x03;
    p_data[1] = 0;
    p_data[2] = 0;
    p_data[3] = 0x00;
    p_data[4] = 0;
    p_data[5] = 0;
    p_data[6] = 0;
    p_data[7] = 0;
}
void motors_prepareInit_3(MOTORS_SELECT_e p_eSelectedMotor, uint8_t* p_data){
    p_data[0] = p_eSelectedMotor & 0x03;
    p_data[1] = 0;
    p_data[2] = 0;
    p_data[3] = 0x01;
    p_data[4] = 0;
    p_data[5] = 0;
    p_data[6] = 0;
    p_data[7] = 0;
}