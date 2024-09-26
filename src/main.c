/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "main.h"

__IO uint16_t adc_value[10];

void rcu_config(void);
void gpio_config(void);
void adc_config(void);
uint16_t adc0_channel_sample(uint8_t channel);
uint16_t adc2_channel_sample(uint8_t channel);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    /* system clocks configuration */
    rcu_config();
    /* systick configuration */
    systick_config();  
    /* GPIO configuration */
    gpio_config();
    /* ADC configuration */
    adc_config();
    /* USART configuration */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART0);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);

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


    BATTERY_init();

    while(1){

        BATTERY_App();
        /* not used on the original firmware PA0 is linked to the charger input voltage*/
        adc_value[0]=adc0_channel_sample(ADC_CHANNEL_0);
        adc_value[1]=adc0_channel_sample(ADC_CHANNEL_1);
        adc_value[2]=adc0_channel_sample(ADC_CHANNEL_10);
        adc_value[3]=adc0_channel_sample(ADC_CHANNEL_11);
        adc_value[4]=adc0_channel_sample(ADC_CHANNEL_12);
        adc_value[5]=adc0_channel_sample(ADC_CHANNEL_13);
        adc_value[6]=adc0_channel_sample(ADC_CHANNEL_14);
        adc_value[7]=adc0_channel_sample(ADC_CHANNEL_15);
        adc_value[8]=adc2_channel_sample(ADC_CHANNEL_4);
        adc_value[9]=adc2_channel_sample(ADC_CHANNEL_6);


        printf("Charger Voltage: %d (%1.2fV)\n", adc_value[0], adc_value[0]* 6.0 * 3.3f / 4095.f);
        printf("Temperature : %d (%1.2fV)\n", adc_value[1], adc_value[1] * 3.3f / 4095.f);
        printf("Battery Voltage: %d (%1.2fV)\n", adc_value[2], adc_value[2] * 10 * 3.3f / 4095.f);
        printf(" DS: %d (%1.2fV)\n", adc_value[3], adc_value[3] * 3.3f / 4095.f);
        printf(" DS bis: %d (%1.2fV)\n", adc_value[7], adc_value[7] * 3.3f / 4095.f);
        printf(" Discharge current: %d (%1.2fV)\n", adc_value[4], adc_value[4] * 3.3f / 4095.f /5.f/0.025f);
        printf(" Charge current: %d (%1.2fV)\n", adc_value[5], adc_value[5] * 3.3f / 4095.f /20.f/0.025f);
        printf(" Mower Motor current: %d (%1.2fV)\n", adc_value[6], adc_value[6] * 3.3f / 4095.f /0.24f);
        printf(" Right Motor current: %d (%1.2fV)\n", adc_value[7], adc_value[7] * 3.3f / 4095.f /0.24f);
        printf(" Left Motor current: %d (%1.2fV)\n", adc_value[8], adc_value[8] * 3.3f / 4095.f /0.24f);
        printf("\n");


        //gpio_bit_reset(GPIOG, GPIO_PIN_10);

        delay_1ms(500);
        /* led blinky*/
        gpio_bit_reset(GPIOF, GPIO_PIN_11);
        delay_1ms(500);
        gpio_bit_set(GPIOF, GPIO_PIN_11);
    } 
}

/*!
    \brief      configure the different system clocks
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_config(void)
{
    /* enable GPIOA clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    /* enable ADC clock */
    rcu_periph_clock_enable(RCU_ADC0);
    rcu_periph_clock_enable(RCU_ADC2);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV6);
}

/*!
    \brief      configure the GPIO peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void gpio_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_GPIOF);
    rcu_periph_clock_enable(RCU_GPIOG);

    /* config the GPIO as analog mode */
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_10MHZ, GPIO_PIN_0 | GPIO_PIN_1);
    /* config the GPIO as analog mode F6 F8 current drive motors */
    gpio_init(GPIOF, GPIO_MODE_AIN, GPIO_OSPEED_10MHZ, GPIO_PIN_6 | GPIO_PIN_8);
    /* config the GPIO as analog mode */
    gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_10MHZ, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 );
  

    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_6 | GPIO_PIN_7);
    gpio_init(GPIOD, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_5);
    gpio_init(GPIOG, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_10);
    gpio_init(GPIOE, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14);
    gpio_init(GPIOF, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_11 | GPIO_PIN_12);
}

/*!
    \brief      configure the ADC peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void adc_config(void)
{
    /* ADC mode config */
    adc_mode_config(ADC_MODE_FREE);
    /*ADC 0*/
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 1U);
    
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE); 
    /* ADC external trigger config */
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);

    /* enable ADC interface */
    adc_enable(ADC0);
    delay_1ms(1U);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);

    /*ADC2*/

     /* ADC data alignment config */
    adc_data_alignment_config(ADC2, ADC_DATAALIGN_RIGHT);
    /* ADC channel length config */
    adc_channel_length_config(ADC2, ADC_REGULAR_CHANNEL, 1U);
    
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC2, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE); 
    /* ADC external trigger config */
    adc_external_trigger_config(ADC2, ADC_REGULAR_CHANNEL, ENABLE);

    /* enable ADC interface */
    adc_enable(ADC2);
    delay_1ms(1U);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC2);
}


/*!
    \brief      ADC0 channel sample
    \param[in]  uint8_t channel
    \param[out] none
    \retval     uint16_t adc_value
*/
uint16_t adc0_channel_sample(uint8_t channel)
{
    /* ADC regular channel config */
    adc_regular_channel_config(ADC0, 0U, channel, ADC_SAMPLETIME_7POINT5);
    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);

    /* wait the end of conversion flag */
    while(!adc_flag_get(ADC0, ADC_FLAG_EOC));
    /* clear the end of conversion flag */
    adc_flag_clear(ADC0, ADC_FLAG_EOC);
    /* return regular channel sample value */
    return (adc_regular_data_read(ADC0));
}

/*!
    \brief      ADC2 channel sample
    \param[in]  uint8_t channel
    \param[out] none
    \retval     uint16_t adc_value
*/
uint16_t adc2_channel_sample(uint8_t channel)
{
    /* ADC regular channel config */
    adc_regular_channel_config(ADC2, 0U, channel, ADC_SAMPLETIME_7POINT5);
    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC2, ADC_REGULAR_CHANNEL);

    /* wait the end of conversion flag */
    while(!adc_flag_get(ADC2, ADC_FLAG_EOC));
    /* clear the end of conversion flag */
    adc_flag_clear(ADC2, ADC_FLAG_EOC);
    /* return regular channel sample value */
    return (adc_regular_data_read(ADC2));
}

/* retarget the gcc's C library printf function to the USART */
#include  <errno.h>
#include  <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
int _write(int file, char *data, int len)
{
   if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
   {
      errno = EBADF;
      return -1;
   }

   // arbitrary timeout 1000
    for(int i=0; i < len; i++) {
        usart_data_transmit(USART0, (uint8_t)data[i]);
        while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    }

   // return # of bytes written - as best we can tell
   return len;
}
