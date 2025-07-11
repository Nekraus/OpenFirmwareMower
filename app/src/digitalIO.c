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
#include "debounce.h"
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
button_t btn_OK;
button_t btn_UP;
button_t btn_DOWN;
button_t btn_RETURN;
button_t btn_HOME;
button_t btn_START;

inputs_t g_inputs = {0};

/* +-----------------------------------------------------------------------+ */
/* |                         Prototype FUNCTIONS                           | */
/* +-----------------------------------------------------------------------+ */
void digitalIO_Init(void);
/* +-----------------------------------------------------------------------+ */
/* |                         PUBLIC FUNCTIONS                              | */
/* +-----------------------------------------------------------------------+ */

/*!
    \brief   void DIGITALIO_App(ULONG thread_input)
    Scan the IO to store it in RAM
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DIGITALIO_App(ULONG thread_input){
    digitalIO_Init();
    while(1){
        if(gpio_input_bit_get(GPIOG, GPIO_PIN_3) == RESET){
            gpio_bit_reset(GPIOG, GPIO_PIN_10);
        }

        g_inputs.start_button = debounce(&btn_START);
        g_inputs.home_button = debounce(&btn_HOME);     
        g_inputs.return_button = debounce(&btn_RETURN);
        g_inputs.up_button = debounce(&btn_UP);
        g_inputs.down_button = debounce(&btn_DOWN);
        g_inputs.ok_button = debounce(&btn_OK);
        g_inputs.left_bumper = gpio_input_bit_get(GPIOC, GPIO_PIN_6) ||
                               gpio_input_bit_get(GPIOG, GPIO_PIN_8); 
        g_inputs.right_bumper = gpio_input_bit_get(GPIOC, GPIO_PIN_7) ||
                                gpio_input_bit_get(GPIOG, GPIO_PIN_2);
        g_inputs.left_up = gpio_input_bit_get(GPIOC, GPIO_PIN_8) ||
                           gpio_input_bit_get(GPIOD, GPIO_PIN_0);
        g_inputs.right_up = gpio_input_bit_get(GPIOC, GPIO_PIN_9) ||
                            gpio_input_bit_get(GPIOA, GPIO_PIN_15);
        g_inputs.charger_connected = gpio_input_bit_get(GPIOD, GPIO_PIN_3) ||
                                     gpio_input_bit_get(GPIOD, GPIO_PIN_6);
        g_inputs.rain_sensor = gpio_input_bit_get(GPIOE, GPIO_PIN_9);
        g_inputs.flip_sensor = gpio_input_bit_get(GPIOA, GPIO_PIN_8);
        g_inputs.estop = gpio_input_bit_get(GPIOG, GPIO_PIN_12)||
                         gpio_input_bit_get(GPIOG, GPIO_PIN_14);

        /*led toggle*/
        gpio_bit_write(GPIOF, GPIO_PIN_11, !gpio_output_bit_get(GPIOF,GPIO_PIN_11));

        tx_thread_sleep(10);
    }
}

/* +-----------------------------------------------------------------------+ */
/* |                           LOCAL FUNCTIONS                             | */
/* +-----------------------------------------------------------------------+ */
/*!
    \brief   void digitalIO_Init(void)
    Init all the periph for the Digital Inputs use
    \param[in]  none
    \param[out] none
    \retval     none
*/
void digitalIO_Init(void){
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
    
    /* Output PD5 to activate the Battery Measure*/
    gpio_init(GPIOD, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_5);
    /* Output PG10 to hold the power */
    gpio_init(GPIOG, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_10);
    /* outputs for enable Power for motors
     PE11 -> 20v, PE13 -> 5v, PE14 -> 3,3v */
    gpio_init(GPIOE, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14);
    /*PF12 -> 12v, PF11 -> LED */
    gpio_init(GPIOF, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_11 | GPIO_PIN_12);

    /*led on*/
    gpio_bit_set(GPIOF, GPIO_PIN_11);

    /* keep the main power active*/
    gpio_bit_set(GPIOG, GPIO_PIN_10);

    /* activate Battery Voltage Measure*/
    gpio_bit_set(GPIOD, GPIO_PIN_5);

    /* activate  12V Motors Drivers */
    gpio_bit_set(GPIOF, GPIO_PIN_12);
    /* activate  5V Motors Drivers */
    gpio_bit_set(GPIOE, GPIO_PIN_13);
    /* activate  3V3 Motors Drivers */
    gpio_bit_set(GPIOE, GPIO_PIN_14);
    /* activate  20v Motors  */
    gpio_bit_set(GPIOE, GPIO_PIN_11);

    init_button(&btn_OK, GPIOG, GPIO_PIN_3);
    init_button(&btn_UP, GPIOG, GPIO_PIN_5);
    init_button(&btn_DOWN, GPIOG, GPIO_PIN_4);
    init_button(&btn_RETURN, GPIOG, GPIO_PIN_6);
    init_button(&btn_HOME, GPIOG, GPIO_PIN_7);
    init_button(&btn_START, GPIOE, GPIO_PIN_12);    
}

void DIGITALIO_GetInputs(inputs_t *p_psInputs){
    memccpy(p_psInputs, &g_inputs, 0, sizeof(inputs_t));
}