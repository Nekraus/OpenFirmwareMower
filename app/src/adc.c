/*
 * adc.c 
 *
 *  Created on: 07/07/2025 
 *      Author: Bruno Lecornu 
 */

/* +-----------------------------------------------------------------------+ */
/* |                               HEADER                                  | */
/* +-----------------------------------------------------------------------+ */
#include "../include/adc.h" 

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
__IO uint16_t adc_value[10];

uint16_t adc0_channel_sample(uint8_t channel);
uint16_t adc2_channel_sample(uint8_t channel);
/* +-----------------------------------------------------------------------+ */
/* |                         Prototype FUNCTIONS                           | */
/* +-----------------------------------------------------------------------+ */
void rcu_config(void);
void gpio_config(void);
void adc_config(void);
void adc_Init(void);
/* +-----------------------------------------------------------------------+ */
/* |                         PUBLIC FUNCTIONS                              | */
/* +-----------------------------------------------------------------------+ */

void ADC_App(ULONG thread_input){

    adc_Init();

    while(1){

        // printf("Charger Voltage: %d (%1.2fV)\n", adc_value[0], adc_value[0]* 6.0 * 3.3f / 4095.f);
        // printf("Temperature : %d (%1.2fV)\n", adc_value[1], adc_value[1] * 3.3f / 4095.f);
        printf("Battery Voltage: %d (%1.2fV)\n", adc_value[2], adc_value[2] * 10 * 3.3f / 4095.f);
        // printf(" DS: %d (%1.2fV)\n", adc_value[3], adc_value[3] * 3.3f / 4095.f);
        // printf(" DS bis: %d (%1.2fV)\n", adc_value[7], adc_value[7] * 3.3f / 4095.f);
        printf(" Discharge current: %d (%1.2fV)\n", adc_value[4], adc_value[4] * 3.3f / 4095.f /5.f/0.025f);
        // printf(" Charge current: %d (%1.2fV)\n", adc_value[5], adc_value[5] * 3.3f / 4095.f /20.f/0.025f);
        printf(" Mower Motor current: %d (%1.2fV)\n", adc_value[6], adc_value[6] * 3.3f / 4095.f /0.24f);
        printf(" Right Motor current: %d (%1.2fV)\n", adc_value[8], adc_value[8] * 3.3f / 4095.f /0.24f);
        printf(" Left Motor current: %d (%1.2fV)\n", adc_value[9], adc_value[9] * 3.3f / 4095.f /0.24f);
        // printf("\n");
        tx_thread_sleep(10);
    }  
}
/* +-----------------------------------------------------------------------+ */
/* |                           LOCAL FUNCTIONS                             | */
/* +-----------------------------------------------------------------------+ */

void adc_Init(void){
    /* GPIO configuration */
    gpio_config();
    /* ADC configuration */
    adc_config();
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
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOF);

    /* config the GPIO as analog mode */
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_10MHZ, GPIO_PIN_0 | GPIO_PIN_1);
    /* config the GPIO as analog mode F6 F8 current drive motors */
    gpio_init(GPIOF, GPIO_MODE_AIN, GPIO_OSPEED_10MHZ, GPIO_PIN_6 | GPIO_PIN_8);
    /* config the GPIO as analog mode */
    gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_10MHZ, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 );
  
}

void adc_timer_config(void)
{
/* enable clock input for Timer1 peripheral */
    rcu_periph_clock_enable(RCU_TIMER1);    
 
    /* get frequency */
    APBx_PSC = (RCU_CFG0 & RCU_CFG0_APB1PSC) >> 8;
    if (0 != (APBx_PSC & 0x04)) {
        clk_src = 2 * rcu_clock_freq_get(CK_APB1);
    } else {
        clk_src =  rcu_clock_freq_get(CK_APB1);
    }

    /* configure TIMER base function */
    timer_parameter_struct timer_initpara;

    timer_initpara.prescaler = clk_src / 1000000 - 1; /*1Mhz*/
    timer_initpara.period = 999; /* 1kHz, 1ms*/
    timer_initpara.repetitioncounter = 0;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_autoreload_value_config(TIMER1, 0);
    timer_init(TIMER1, &timer_initpara);

    timer_enable(TIMER1);
}
/*!
    \brief      configure the ADC peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void adc_config(void)
{
    /* enable ADC clock */
    rcu_periph_clock_enable(RCU_ADC0);
    rcu_periph_clock_enable(RCU_ADC2);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV6);

    /* ADC mode config */
    adc_mode_config(ADC_MODE_FREE);
    /*ADC 0*/
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 8U);
    /* ADC scan mode */
    adc_special_function_config(ADC0 , ADC_SCAN_MODE, ENABLE);
    adc_dma_mode_enable(ADC2);
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_EXTTRIG_REGULAR_T1_CH1); 

    /*ADC2*/

    /* ADC data alignment config */
    adc_data_alignment_config(ADC2, ADC_DATAALIGN_RIGHT);
    /* ADC channel length config */
    adc_channel_length_config(ADC2, ADC_REGULAR_CHANNEL, 2U);
    /* ADC scan mode */
    adc_special_function_config(ADC2 , ADC_SCAN_MODE, ENABLE);
    adc_dma_mode_enable(ADC2);
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC2, ADC_REGULAR_CHANNEL, ADC2_EXTTRIG_REGULAR_T1_CH2); 

    /* ADC regular channel config */
    adc_regular_channel_config(ADC0, 0U, ADC_CHANNEL_0, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC0, 1U, ADC_CHANNEL_1, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC0, 2U, ADC_CHANNEL_10, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC0, 3U, ADC_CHANNEL_11, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC0, 4U, ADC_CHANNEL_12, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC0, 5U, ADC_CHANNEL_13, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC0, 6U, ADC_CHANNEL_14, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC0, 7U, ADC_CHANNEL_15, ADC_SAMPLETIME_239POINT5);

    adc_regular_channel_config(ADC2, 0U, ADC_CHANNEL_4, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC2, 1U, ADC_CHANNEL_6, ADC_SAMPLETIME_239POINT5);

    /* enable ADC interface */
    adc_enable(ADC0);
    adc_enable(ADC2);
    tx_thread_sleep(1);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);
    adc_calibration_enable(ADC2);

    /* ADC external trigger config */
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);
    adc_external_trigger_config(ADC2, ADC_REGULAR_CHANNEL, ENABLE);
 
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

/* +-----------------------------------------------------------------------+ */ 
/* |                               END OF FILE                             | */
/* +-----------------------------------------------------------------------+ */