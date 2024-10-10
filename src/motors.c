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

typedef enum {
    MOTORS_CMDID_TICKS = 0x00,
    MOTORS_CMDID_SPEED = 0x02,
    MOTORS_CMDID_CURRENT = 0x03,
    MOTORS_SELECT_STOP = 0x05,
} MOTORS_CMDID_e;


typedef struct {
uint8_t u8Motor_id;
uint8_t u8Cmd_id;
uint8_t pu8Data[6];
} motor_msg_tx_t;
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
void motors_motors_SendSpeedCmd(MOTORS_SELECT_e p_eSelectedMotor, int16_t p_s16Speed, int16_t p_s16Voltage);
void motors_SendBrakeCmd(MOTORS_SELECT_e p_eSelectedMotor);
void motors_SendAskCurrentCmd(MOTORS_SELECT_e p_eSelectedMotor);
void motors_SendMsg(motor_msg_tx_t* p_psMsg);
uint8_t motors_Calculatechecksum(motor_msg_tx_t* p_psMsg);
void motors_prepareInit_1(MOTORS_SELECT_e p_eSelectedMotor);
void motors_prepareInit_2(MOTORS_SELECT_e p_eSelectedMotor);
void motors_prepareInit_3(MOTORS_SELECT_e p_eSelectedMotor);

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
    //nvic_irq_enable(DMA0_Channel6_IRQn, 0, 1);
    //nvic_irq_enable(USART1_IRQn, 0, 1);

    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_3);

    usart_deinit(USART1);
    usart_baudrate_set(USART1, 115200U);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
   // usart_interrupt_enable(USART1, USART_INT_IDLE);
    usart_enable(USART1);
    
    /* deinitialize DMA channel6(USART1 tx) */
    dma_deinit(DMA0, DMA_CH6);
    dma_struct_para_init(&dma_init_struct);

    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr = (uint32_t)txbuffer;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = 0;
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
    usart_dma_receive_config(USART1, USART_DENR_ENABLE);
    /* enable DMA0 channel4 transfer complete interrupt */
    dma_interrupt_enable(DMA0, DMA_CH5, DMA_INT_FTF);
    /* enable DMA0 channel4 */
    dma_channel_enable(DMA0, DMA_CH5);
    /* enable USART DMA for transmission */
    usart_dma_transmit_config(USART1, USART_DENT_ENABLE);
    // /* enable DMA0 channel3 transfer complete interrupt */
    // dma_interrupt_enable(DMA0, DMA_CH6, DMA_INT_FTF);
    // /* enable DMA0 channel3 */
    // dma_channel_enable(DMA0, DMA_CH6);

    /* Mux Control */
    gpio_init(GPIOE, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
    gpio_init(GPIOE, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
}

void MOTORS_App(void){
    static int state = 0;

    switch (state)
    {
    case 0:
        motors_selectMotor(MOTORS_SELECT_RIGHT);
        motors_prepareInit_1(MOTORS_SELECT_RIGHT);
        delay_1ms(10);
        motors_prepareInit_2(MOTORS_SELECT_RIGHT);
        delay_1ms(10);
        motors_prepareInit_3(MOTORS_SELECT_RIGHT);
        delay_1ms(10);
        motors_selectMotor(MOTORS_SELECT_LEFT);
        delay_1ms(1);
        motors_prepareInit_1(MOTORS_SELECT_LEFT);
        delay_1ms(10);
        motors_prepareInit_2(MOTORS_SELECT_LEFT);
        delay_1ms(10);
        motors_prepareInit_3(MOTORS_SELECT_LEFT);
        delay_1ms(10);
        state = 1;
        break;
    
        case 1:
        motors_selectMotor(MOTORS_SELECT_RIGHT);
        delay_1ms(1);
        motors_motors_SendSpeedCmd(MOTORS_SELECT_RIGHT, 2000 ,0);
        delay_1ms(10);
        motors_selectMotor(MOTORS_SELECT_LEFT);
        delay_1ms(1);
        motors_motors_SendSpeedCmd(MOTORS_SELECT_LEFT, 000, 0);
        delay_1ms(10);
        break;
    
    default:
        break;
    }
}

void MOTORS_DMARxIRQ(void){
    volatile tmp;
    if(rxbuffer[4] == 0x03){
        tmp = 1;
    }
    dma_channel_disable(DMA0, DMA_CH5);
    DMA_CH5CNT(DMA0) = RX_SIZE;
    dma_channel_enable(DMA0, DMA_CH5);
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
        switch (p_eSelectedMotor)
        {
        case   MOTORS_SELECT_RIGHT :
            gpio_bit_reset(GPIOE,GPIO_PIN_7);
            gpio_bit_set(GPIOE,GPIO_PIN_8);
            break;
        case   MOTORS_SELECT_LEFT :
            gpio_bit_reset(GPIOE,GPIO_PIN_7);
            gpio_bit_reset(GPIOE,GPIO_PIN_8);
            break;
        case   MOTORS_SELECT_MOWER :
            gpio_bit_set(GPIOE,GPIO_PIN_7);
            gpio_bit_set(GPIOE,GPIO_PIN_8);
            break;
        
        default:
            break;
        }
}

/*!
    \brief   motors_SendBrakeCmd
    Send Speed command to motors Drivers 
    \param[in]  int16_t p_s16Speed
    \param[in]  int16_t p_s16Voltage
    \param[out] none
    \retval     SUCCESS FAILED
*/
void motors_SendBrakeCmd(MOTORS_SELECT_e p_eSelectedMotor){
    motor_msg_tx_t tmp;
    if(p_eSelectedMotor < MOTORS_SELECT_MAX){
        tmp.u8Motor_id = p_eSelectedMotor;
        tmp.u8Cmd_id = 0x05;
        tmp.pu8Data[0] = 0 ;
        tmp.pu8Data[1] = 0 ;
        tmp.pu8Data[2] = 0 ;
        tmp.pu8Data[3] = 0 ;
        tmp.pu8Data[4] = 0 ;
        tmp.pu8Data[5] = 0 ;
        motors_SendMsg(&tmp);
    }
}

void motors_SendAskCurrentCmd(MOTORS_SELECT_e p_eSelectedMotor){
    motor_msg_tx_t tmp;
    if(p_eSelectedMotor < MOTORS_SELECT_MAX){
        tmp.u8Motor_id = p_eSelectedMotor;
        tmp.u8Cmd_id = 0x03;
        tmp.pu8Data[0] = 0 ;
        tmp.pu8Data[1] = 0 ;
        tmp.pu8Data[2] = 0 ;
        tmp.pu8Data[3] = 0 ;
        tmp.pu8Data[4] = 0 ;
        tmp.pu8Data[5] = 0 ;
        motors_SendMsg(&tmp);
    }
}

/*!
    \brief   motors_SendSpeedCmd
    Send Speed command to motors Drivers 
    \param[in]  int16_t p_s16Speed
    \param[in]  int16_t p_s16Voltage
    \param[out] none
    \retval     SUCCESS FAILED
*/
void motors_motors_SendSpeedCmd(MOTORS_SELECT_e p_eSelectedMotor, int16_t p_s16Speed, int16_t p_s16Voltage){
    motor_msg_tx_t tmp;
    if(p_eSelectedMotor < MOTORS_SELECT_MAX){
        tmp.u8Motor_id = p_eSelectedMotor;
        tmp.u8Cmd_id = MOTORS_CMDID_SPEED;
        tmp.pu8Data[0] = 0 ;
        tmp.pu8Data[1] = 0 ;
        tmp.pu8Data[2] = (uint8_t)(p_s16Voltage >> 8) ;
        tmp.pu8Data[3] = (uint8_t)(p_s16Voltage & 0x00FF) ;
        tmp.pu8Data[4] = (uint8_t)(p_s16Speed >> 8) ;
        tmp.pu8Data[5] = (uint8_t)(p_s16Speed & 0x00FF) ;
        motors_SendMsg(&tmp);
    }
}

/*!
    \brief   motors_SendMsg
    take the msg add headers and CRC and send by DMA to UART
    \param[in]  motor_msg_tx_t p_psMsg
    \param[out] none
    \retval     SUCCESS FAILED
*/
void motors_SendMsg(motor_msg_tx_t* p_psMsg){
    if(p_psMsg == NULL){
        return -1;
    }
    else{
        txbuffer[0]  = 0xD5;
        txbuffer[1]  = 0xE5;
        txbuffer[2]  = p_psMsg->u8Motor_id;
        txbuffer[3]  = p_psMsg->pu8Data[2];
        txbuffer[4]  = p_psMsg->u8Cmd_id;
        txbuffer[5]  = p_psMsg->pu8Data[3];
        txbuffer[6]  = motors_Calculatechecksum(p_psMsg);
        txbuffer[7]  = p_psMsg->pu8Data[4];
        txbuffer[8]  = p_psMsg->pu8Data[0];
        txbuffer[9]  = p_psMsg->pu8Data[5];
        txbuffer[10] = p_psMsg->pu8Data[1];
       
        if(DMA_CH6CNT(DMA0) == 0 ){
        dma_channel_disable(DMA0, DMA_CH6);
        DMA_CH6CNT(DMA0) = TX_SIZE;
        dma_channel_enable(DMA0, DMA_CH6);
        }

        return 0;
    }
}

/*!
    \brief   void motors_Calculatechecksum(uint8_t* p_pu8Buffer)
    Calculate the CRC for UART communication 
    \param[in]  uint8_t* p_pu8Buffer buffer to send or received 
    \param[out] none
    \retval     u8 CRC
*/
uint8_t motors_Calculatechecksum(motor_msg_tx_t* p_psMsg){
    if(p_psMsg == NULL){
        return 0;
    }
    else{
        return ((p_psMsg->u8Cmd_id + p_psMsg->u8Motor_id + p_psMsg->pu8Data[0] + p_psMsg->pu8Data[1] + \
                p_psMsg->pu8Data[2] + p_psMsg->pu8Data[3] + p_psMsg->pu8Data[4] + p_psMsg->pu8Data[5]   \
        ) & 0x7f );
    }

}

void motors_prepareInit_1(MOTORS_SELECT_e p_eSelectedMotor){
    motor_msg_tx_t tmp;
    tmp.u8Motor_id = p_eSelectedMotor;
    tmp.u8Cmd_id = 0x04;
    tmp.pu8Data[0] = 0 ;
    tmp.pu8Data[1] = 0 ;
    tmp.pu8Data[2] = 0 ;
    tmp.pu8Data[3] = 0 ;
    tmp.pu8Data[4] = 0 ;
    tmp.pu8Data[5] = 0 ;
    motors_SendMsg(&tmp);
}
void motors_prepareInit_2(MOTORS_SELECT_e p_eSelectedMotor){
    motor_msg_tx_t tmp;
    tmp.u8Motor_id = p_eSelectedMotor;
    tmp.u8Cmd_id = 0x00;
    tmp.pu8Data[0] = 0 ;
    tmp.pu8Data[1] = 0 ;
    tmp.pu8Data[2] = 0 ;
    tmp.pu8Data[3] = 0 ;
    tmp.pu8Data[4] = 0 ;
    tmp.pu8Data[5] = 0 ;
    motors_SendMsg(&tmp);
}
void motors_prepareInit_3(MOTORS_SELECT_e p_eSelectedMotor){
    motor_msg_tx_t tmp;
    tmp.u8Motor_id = p_eSelectedMotor;
    tmp.u8Cmd_id = 0x01;
    tmp.pu8Data[0] = 0 ;
    tmp.pu8Data[1] = 0 ;
    tmp.pu8Data[2] = 0 ;
    tmp.pu8Data[3] = 0 ;
    tmp.pu8Data[4] = 0 ;
    tmp.pu8Data[5] = 0 ;
    motors_SendMsg(&tmp);
}