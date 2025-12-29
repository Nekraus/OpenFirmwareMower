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
uint16_t adc0_value[8];
uint16_t adc2_value[2];

uint16_t
adc0_channel_sample(uint8_t channel);
uint16_t adc2_channel_sample(uint8_t channel);
/* +-----------------------------------------------------------------------+ */
/* |                         Prototype FUNCTIONS                           | */
/* +-----------------------------------------------------------------------+ */
void adc_timer_config(void);
void gpio_config(void);
void adc_config(void);
void adc_dma_config(void);
void adc_Init(void);
/* +-----------------------------------------------------------------------+ */
/* |                         PUBLIC FUNCTIONS                              | */
/* +-----------------------------------------------------------------------+ */

void ADC_App(ULONG thread_input)
{

  adc_Init();

  while (1)
  {

    // printf("Charger Voltage: %d (%1.2fV)\n", adc0_value[0], adc0_value[0]* 6.0 * 3.3f / 4095.f);
    // printf("Temperature : %d (%1.2fV)\n", adc0_value[1], adc0_value[1] * 3.3f / 4095.f);
    printf("Battery Voltage: %d (%1.2fV)\n", adc0_value[2], adc0_value[2] * 10 * 3.3f / 4095.f);
    // printf(" DS: %d (%1.2fV)\n", adc0_value[3], adc0_value[3] * 3.3f / 4095.f);
    // printf(" DS bis: %d (%1.2fV)\n", adc0_value[7], adc0_value[7] * 3.3f / 4095.f);
    printf(" Discharge current: %d (%1.2fV)\n", adc0_value[4], adc0_value[4] * 3.3f / 4095.f / 5.f / 0.025f);
    // printf(" Charge current: %d (%1.2fV)\n", adc0_value[5], adc0_value[5] * 3.3f / 4095.f /20.f/0.025f);
    printf(" Mower Motor current: %d (%1.2fV)\n", adc0_value[6], adc0_value[6] * 3.3f / 4095.f / 0.24f);
    printf(" Right Motor current: %d (%1.2fV)\n", adc2_value[8], adc2_value[0] * 3.3f / 4095.f / 0.24f);
    printf(" Left Motor current: %d (%1.2fV)\n", adc2_value[9], adc2_value[1] * 3.3f / 4095.f / 0.24f);
    // printf("\n");
    tx_thread_sleep(10);
  }
}
/* +-----------------------------------------------------------------------+ */
/* |                           LOCAL FUNCTIONS                             | */
/* +-----------------------------------------------------------------------+ */

void adc_Init(void)
{
  /* GPIO configuration */
  gpio_config();
  /* ADC timer configuration */
  adc_timer_config();
  /* ADC configuration */
  adc_config();
  /* DMA configuration*/
  adc_dma_config();
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
  gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_10MHZ, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
}

void adc_timer_config(void)
{
  uint32_t APBx_PSC = 0;
  uint32_t clk_src = 0;
  /* enable clock input for Timer1 peripheral */
  rcu_periph_clock_enable(RCU_TIMER1);

  /* get frequency */
  APBx_PSC = (RCU_CFG0 & RCU_CFG0_APB1PSC) >> 8;
  if (0 != (APBx_PSC & 0x04))
  {
    clk_src = 2 * rcu_clock_freq_get(CK_APB1);
  }
  else
  {
    clk_src = rcu_clock_freq_get(CK_APB1);
  }

  /* configure TIMER base function */
  timer_parameter_struct timer_initpara;

  timer_initpara.prescaler = clk_src / 1000000 - 1; /*1Mhz*/
  timer_initpara.period = 999;                      /* 1kHz, 1ms*/
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
  adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
  adc_dma_mode_enable(ADC2);
  /* ADC trigger config */
  adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_EXTTRIG_REGULAR_T1_CH1);

  /*ADC2*/

  /* ADC data alignment config */
  adc_data_alignment_config(ADC2, ADC_DATAALIGN_RIGHT);
  /* ADC channel length config */
  adc_channel_length_config(ADC2, ADC_REGULAR_CHANNEL, 2U);
  /* ADC scan mode */
  adc_special_function_config(ADC2, ADC_SCAN_MODE, ENABLE);
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

  /* ADC DMA function enable */
  adc_dma_mode_enable(ADC0);
  adc_dma_mode_enable(ADC2);

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

void adc_dma_config(void)
{
  /* enable DMA0 clock */
  rcu_periph_clock_enable(RCU_DMA0);
  rcu_periph_clock_enable(RCU_DMA1);
  /* ADC_DMA_channel configuration */
  dma_parameter_struct dma_data_parameter;

  /* ADC DMA_channel configuration */
  dma_deinit(DMA0, DMA_CH0);

  /* initialize DMA single data mode */
  dma_data_parameter.periph_addr = (uint32_t)(&ADC_RDATA(ADC0));
  dma_data_parameter.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
  dma_data_parameter.memory_addr = (uint32_t)(&adc0_value);
  dma_data_parameter.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
  dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
  dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_16BIT;
  dma_data_parameter.direction = DMA_PERIPHERAL_TO_MEMORY;
  dma_data_parameter.number = 8;
  dma_data_parameter.priority = DMA_PRIORITY_HIGH;
  dma_init(DMA0, DMA_CH0, &dma_data_parameter);

  /* ADC DMA_channel configuration */
  dma_deinit(DMA1, DMA_CH4);

  /* initialize DMA single data mode */
  dma_data_parameter.periph_addr = (uint32_t)(&ADC_RDATA(ADC2));
  dma_data_parameter.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
  dma_data_parameter.memory_addr = (uint32_t)(&adc2_value);
  dma_data_parameter.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
  dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
  dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_16BIT;
  dma_data_parameter.direction = DMA_PERIPHERAL_TO_MEMORY;
  dma_data_parameter.number = 2;
  dma_data_parameter.priority = DMA_PRIORITY_HIGH;
  dma_init(DMA1, DMA_CH4, &dma_data_parameter);

  /* enable DMA channel */
  dma_channel_enable(DMA0, DMA_CH0);
  dma_channel_enable(DMA1, DMA_CH4);
}

/*!
    \brief      ADC0 channel sample
    \param[in]  uint8_t channel
    \param[out] none
    \retval     uint16_t adc0_value
*/
uint16_t adc0_channel_sample(uint8_t channel)
{
  /* ADC regular channel config */
  adc_regular_channel_config(ADC0, 0U, channel, ADC_SAMPLETIME_7POINT5);
  /* ADC software trigger enable */
  adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);

  /* wait the end of conversion flag */
  while (!adc_flag_get(ADC0, ADC_FLAG_EOC))
    ;
  /* clear the end of conversion flag */
  adc_flag_clear(ADC0, ADC_FLAG_EOC);
  /* return regular channel sample value */
  return (adc_regular_data_read(ADC0));
}

/*!
    \brief      ADC2 channel sample
    \param[in]  uint8_t channel
    \param[out] none
    \retval     uint16_t adc2_value
*/
uint16_t adc2_channel_sample(uint8_t channel)
{
  /* ADC regular channel config */
  adc_regular_channel_config(ADC2, 0U, channel, ADC_SAMPLETIME_7POINT5);
  /* ADC software trigger enable */
  adc_software_trigger_enable(ADC2, ADC_REGULAR_CHANNEL);

  /* wait the end of conversion flag */
  while (!adc_flag_get(ADC2, ADC_FLAG_EOC))
    ;
  /* clear the end of conversion flag */
  adc_flag_clear(ADC2, ADC_FLAG_EOC);
  /* return regular channel sample value */
  return (adc_regular_data_read(ADC2));
}

/* +-----------------------------------------------------------------------+ */
/* |                               END OF FILE                             | */
/* +-----------------------------------------------------------------------+ */