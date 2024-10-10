/*
 * digitalIO.c 
 *
 *  Created on: 08/10/2024 
 *      Author: Bruno Lecornu 
 */

/* +-----------------------------------------------------------------------+ */
/* |                               HEADER                                  | */
/* +-----------------------------------------------------------------------+ */
#include "digitalIO.h" 

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

/*!
    \brief   void DIGITALIO_Init(void)
    Init all the periph for the Digital Inputs use
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DIGITALIO_Init(void){
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_GPIOG);

    /* config the buttons and estop 
    * PG2 -> Right Bumper bis, PG8 -> Left Bumper bis,
    * PG3 -> OK, PG4 -> DOWN, PG5 -> UP, PG6 -> Return, PG7 -> Home
    * PG12 -> STOP1, PG14 -> STOP2
    */
    gpio_init(GPIOG, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_10MHZ, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5| \
    GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_12| GPIO_PIN_14);
    /* config the Rain sensor and Start button 
    * PE9 -> Rain sensor, PE12 -> Start button
    */
    gpio_init(GPIOE, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_10MHZ, GPIO_PIN_9|GPIO_PIN_12);
    /* config Charger Connected pin 
    * PD0 -> Left Up bis ,PD3 -> Charger connected, PD6 -> Charger connected bis
    */
    gpio_init(GPIOD, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_10MHZ, GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_6);
    /* Bumper & Up sensor 
    * PC6 -> Left Bumper, PC7 -> Right Bumper, PC8 -> Left Up, PC9 -> Right Up
    */
    gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_10MHZ, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9); 
    /* Flip sensor 
    * PA8 -> Flip sensor, PA15 -> Right Up bis
    */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_10MHZ, GPIO_PIN_8|GPIO_PIN_15);

    /* Outputs for the stop sensors 
    * PG11 -> Activate STOP1, PG13 -> Activate STOP2
    */
    gpio_init(GPIOG, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_11|GPIO_PIN_13);
}

/* +-----------------------------------------------------------------------+ */
/* |                           LOCAL FUNCTIONS                             | */
/* +-----------------------------------------------------------------------+ */
