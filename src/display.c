/*
 * display.c 
 *
 *  Created on: 14/11/2024 
 *      Author: Bruno Lecornu 
 */

/* +-----------------------------------------------------------------------+ */
/* |                               HEADER                                  | */
/* +-----------------------------------------------------------------------+ */
#include "display.h" 

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
char* hello = "Hello";
char* world = "World";
/* +-----------------------------------------------------------------------+ */
/* |                         Prototype FUNCTIONS                           | */
/* +-----------------------------------------------------------------------+ */

/* +-----------------------------------------------------------------------+ */
/* |                         PUBLIC FUNCTIONS                              | */
/* +-----------------------------------------------------------------------+ */
/*!
    \brief   void DISPLAY_Init(void)
    Init all the periph for the Digital Inputs use
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DISPLAY_Init(void){
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOD);

    /*PA11 -> VE ?? PA12 -> LED + ??*/
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_MAX, GPIO_PIN_11 | GPIO_PIN_12);
    gpio_bit_reset(GPIOA, GPIO_PIN_11);

    /* PB12?? PB13 -> RS PB14-> Read/Write PB15 -> E */
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_MAX, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

    gpio_init(GPIOD, GPIO_MODE_OUT_PP, GPIO_OSPEED_MAX, GPIO_PIN_8 |GPIO_PIN_9 |GPIO_PIN_10|GPIO_PIN_11| \
                                                        GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

    
    delay_1ms(5);
    gpio_bit_reset(GPIOA, GPIO_PIN_12);
    delay_1ms(20);
    gpio_bit_set(GPIOA, GPIO_PIN_12);
    delay_1ms(20);
    /* Function Set 8 bit 2 lines */
    display_send_cmd(0x39);
    delay_1ms(5);
    /* Function Set 8 bit 2 lines */
    display_send_cmd(0x39);
    delay_1ms(5);
    /* Cursor On */
    display_send_cmd(0x14);
    /* Set CGRAM address 0x14? */
    display_send_cmd(0x54);
    /* Set CGRAM address 0x2C */
    display_send_cmd(0x6c);
    /* Set CGRAM address 0x3e */
    display_send_cmd(0x7e);
    /* Display ON */
    display_send_cmd(0xc);
    delay_1ms(5);
    /* clear display */
    display_send_cmd(1);
    delay_1ms(10);
    /* Entry Mode Set increment */
    display_send_cmd(6);
    delay_1ms(10);

    display_writeStringLineColumn(hello,0,0);
    display_writeStringLineColumn(world,1,0);

}

/* +-----------------------------------------------------------------------+ */
/* |                           LOCAL FUNCTIONS                             | */
/* +-----------------------------------------------------------------------+ */

void display_send_cmd(uint8_t p_u8Cmd){
    /* Reset PB13 RS - PB14 R/W*/
    gpio_bit_reset(GPIOB, GPIO_PIN_13 | GPIO_PIN_14);
    /* Set PB15 E */
    gpio_bit_set(GPIOB, GPIO_PIN_15);
    /* send Data */
    gpio_port_write(GPIOD, p_u8Cmd<<8 );
    /*todo change wait*/
    delay_1ms(1);
    gpio_bit_reset(GPIOB, GPIO_PIN_15);
    /*todo change wait*/
    delay_1ms(1);
}

void display_send_data(uint8_t p_u8data){
    /* Reset PB14 R/W*/
    gpio_bit_reset(GPIOB, GPIO_PIN_14);
    /* Set PB13 RS PB15 E */
    gpio_bit_set(GPIOB, GPIO_PIN_13 | GPIO_PIN_15);
    /* send Data */
    gpio_port_write(GPIOD, p_u8data<<8 );
    /*todo change wait*/
    delay_1ms(1);
    gpio_bit_reset(GPIOB, GPIO_PIN_15);
    /*todo change wait*/
    delay_1ms(1);
}


void display_setCursor( uint8_t p_u8Line, uint8_t p_u8Col){
  /* 16 x 2 display */
  uint8_t   l_u8tmp = p_u8Col & 0xf;
  /* Set  DDRAM address to 0x00 and return cursor to its original position */
  display_send_cmd(2);
  if ((p_u8Line & 1) != 0) {
    l_u8tmp = l_u8tmp + 0x40;
  }
  display_send_cmd(l_u8tmp);
}

void display_writeCharLineColum(char p_u8Char, uint8_t p_u8Line, uint8_t p_u8Col){
    display_setCursor(p_u8Line, p_u8Col);
    display_send_data(p_u8Char);
}

void display_writeStringLineColumn(char* p_u8String, uint8_t p_u8Line, uint8_t p_u8Col){
    display_setCursor(p_u8Line, p_u8Col);
    while(p_u8String != '/0'){
         display_send_data(*p_u8String++);
    }
}