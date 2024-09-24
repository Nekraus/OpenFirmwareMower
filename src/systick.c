/*!
    \file    systick.c
    \brief   the systick configuration file

*/


#include "gd32f30x.h"
#include "systick.h"

volatile static uint32_t delay;
volatile static uint32_t ticks;

/*!
    \brief      configure systick
    \param[in]  none
    \param[out] none
    \retval     none
*/
void systick_config(void)
{
    /* setup systick timer for 1000Hz interrupts */
    if (SysTick_Config(SystemCoreClock / 1000U)){
        /* capture error */
        while (1){
        }
    }
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
}

/*!
    \brief      delay a time in milliseconds
    \param[in]  count: count in milliseconds
    \param[out] none
    \retval     none
*/
void delay_1ms(uint32_t count)
{
    delay = count;

    while(0U != delay){
    }
}

/*!
    \brief      delay decrement
    \param[in]  none
    \param[out] none
    \retval     none
*/
void delay_decrement(void)
{
    if (0U != delay){
        delay--;
    }
}

/*!
    \brief      ticks_increment
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ticks_increment(void){
    ticks++;
}

/*!
    \brief      get_ticks
    \param[in]  none
    \param[out] none
    \retval     ticks in ms
*/
uint32_t get_ticks(void){
    return ticks;
}