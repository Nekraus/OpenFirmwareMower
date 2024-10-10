#include "gd32f30x.h"
#include "systick.h"

typedef enum 
{
    MODE_CS_IDLE,
    MODE_CS_CHARGE,
    MODE_DS,
    MODE_MAX
} battery_mode_e;

typedef enum 
{
    CS_STATE_OFF,
    CS_STATE_INIT,
    CS_STATE_WAIT_NEXT_TRAME,
    CS_STATE_WAIT_FIRST_BIT,
    CS_STATE_READING
} battery_CS_STATE_e;


__IO uint32_t battery_u32StartTime_AnalogWDG = 0;
__IO uint32_t battery_u32PulseTime_AnalogWDG = 0;
__IO uint32_t battery_u32NbPulse = 0;

__IO battery_CS_STATE_e battery_CSState = CS_STATE_OFF;
__IO uint32_t battery_u32StartTime_CS = 0;
__IO uint32_t battery_u32StatusReceived = 0;
__IO uint8_t  battery_pu8Status[5] = {0};
__IO uint8_t  battery_u8StatusNbBit = 0;

const uint32_t battery_cu32TimeoutPulse = 750;
const uint32_t battery_cu32WaitError = 50;
const uint32_t battery_cu32ReactivateWDG = 200;

uint8_t battery_get_DS_State_new(uint16_t p_u16PC1, uint8_t p_u8TypeBattery);
uint8_t battery_get_DS_State_old(uint16_t p_u16PC1, uint8_t p_u8TypeBattery);
int32_t battery_Set_Mode(battery_mode_e p_eMode);

/*!
    \brief      BATTERY init function
    Init the ADC to count the pulse on DS
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BATTERY_init(void)
{
    uint32_t APBx_PSC = 0;
    uint32_t clk_src = 0;
    /* Set the trigger limit */
    adc_watchdog_threshold_config(ADC0, 0, 3350);

    /* TODO dedicated function */
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    nvic_irq_enable(ADC0_1_IRQn, 0, 0);
    nvic_irq_enable(TIMER2_IRQn, 2, 2); 
    /* enable and set key EXTI interrupt to the lowest priority */
    nvic_irq_enable(EXTI5_9_IRQn, 2U, 0U);

    /* enable clock input for Timer2 peirpheral */
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_TIMER2);    
 
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
    timer_initpara.period = 499; /* 2kHz, 500 µs*/
    timer_initpara.repetitioncounter = 0;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_autoreload_value_config(TIMER2, 0);
    timer_init(TIMER2, &timer_initpara);
    /* enable timer update interrupt */ 
    /* happens when timer is overflowing (period / reload value exceeded) and it is set back to 0 again */
    timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(TIMER2, TIMER_INT_UP);
    timer_enable(TIMER2);

    /* enable the key clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    /* configure button pin as input */
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_8);

    /* connect key EXTI line to key GPIO pin */
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOB, GPIO_PIN_SOURCE_8);
    /* configure key EXTI line */
    exti_init(EXTI_8, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_disable(EXTI_8);
    exti_interrupt_flag_clear(EXTI_8);
}

/*!
    \brief      BATTERY_ActiveAnalogWatchdog
    Activate the analog watchdog on ADC0 on PC1
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BATTERY_ActiveAnalogWatchdog(void)
{
    if ((ADC_CTL0(ADC0) & ADC_CTL0_RWDEN) == 0)
    {
        battery_u32StartTime_AnalogWDG = get_ticks();
        battery_u32PulseTime_AnalogWDG = battery_u32StartTime_AnalogWDG;

        /* activate interrupt ADC interrupt */
        adc_interrupt_enable(ADC0, ADC_INT_WDE);
        /* PC1 (C11) -> DS  Channel */
        adc_watchdog_single_channel_enable(ADC0, ADC_CHANNEL_11);
    }
}

/*!
    \brief      BATTERY_DesactiveAnalogWatchdog
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BATTERY_DesactiveAnalogWatchdog(void)
{
    /* desactivate interrupt ADC interrupt */
    adc_interrupt_disable(ADC0, ADC_INT_WDE);
    /* disable analog WDG */
    adc_watchdog_disable(ADC0);
}

/*!
    \brief      BATTERY_AnalogWatchdogIRQ
    Need to be set in the ADC IRQ, function to count pulses on DS
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BATTERY_AnalogWatchdogIRQ(void)
{
    battery_u32PulseTime_AnalogWDG = get_ticks();
    battery_u32NbPulse++;
}
/*!
    \brief              BATTERY_ExtlineIRQ();
    Need to be set in the Extline IRQ, function to read the UART stye data on CS
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BATTERY_ExtlineIRQ(void){
    if(battery_CSState == CS_STATE_INIT || battery_CSState == CS_STATE_WAIT_NEXT_TRAME){
        /* if a trame is received, it will reset the timer autoreaload register until the end of the frame */
        battery_CSState = CS_STATE_WAIT_NEXT_TRAME;
        timer_counter_value_config(TIMER2, 0);
        timer_autoreload_value_config(TIMER2, 49999);
        timer_enable(TIMER2);
    }
    else if(battery_CSState == CS_STATE_WAIT_FIRST_BIT){
        battery_CSState = CS_STATE_READING;
        /* set Timer for the next bit, bit is detected if falling edge is detected or the timer elasps */
        timer_counter_value_config(TIMER2, 0);
        timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
        timer_autoreload_value_config(TIMER2, 1040);
        timer_enable(TIMER2);
        uint8_t l_u8Bit = gpio_input_bit_get(GPIOB, GPIO_PIN_8);
        battery_pu8Status[0] = (l_u8Bit & 0x01) << 7;
        battery_u8StatusNbBit = 1;
    }
    else if(battery_CSState == CS_STATE_READING){
        timer_counter_value_config(TIMER2, 0);
        timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
        uint8_t l_u8Bit = gpio_input_bit_get(GPIOB, GPIO_PIN_8);
        /* divide by 8 (>>3) to get Bytes */
        battery_pu8Status[(battery_u8StatusNbBit>>3)] |=(l_u8Bit & 0x01) << (7 - (battery_u8StatusNbBit & 7) & 0xff)  ;
        battery_u8StatusNbBit ++;
    }
    else
    {
        /* nothing here */
    }
}

/*!
    \brief BATTERY_ExtlineIRQ();
    Need to be set in the Timer IRQ, function to read the UART stye data on CS
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BATTERY_TimerIRQ(void)
{
 if( battery_CSState == CS_STATE_WAIT_NEXT_TRAME)
 {   
    battery_CSState = CS_STATE_WAIT_FIRST_BIT;
 }
 else if(battery_CSState == CS_STATE_READING)
 {
    uint8_t l_u8Bit = gpio_input_bit_get(GPIOB, GPIO_PIN_8);
    /* divide by 8 (>>3) to get Bytes */
    battery_pu8Status[(battery_u8StatusNbBit>>3)] |=(l_u8Bit & 0x01) << (7 - (battery_u8StatusNbBit & 7) & 0xff) ;
    battery_u8StatusNbBit ++;
    if( 39 < battery_u8StatusNbBit )
    {
        battery_u32StatusReceived = 1;
        battery_CSState = CS_STATE_WAIT_FIRST_BIT;
        timer_disable(TIMER2);

    }

 }
 else{
    /* nothing here */
 }
}

/*!
    \brief              BATTERY_App();
    all the logic to read the battery data on CS line
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BATTERY_App(void){
    /* Set CS */
    battery_Set_Mode(MODE_DS);
    if(battery_u32StatusReceived == 1){
        battery_u32StatusReceived = 0;
        printf("Bat Stat: %x%x %x%x\n", battery_pu8Status[3], battery_pu8Status[4], battery_pu8Status[1], battery_pu8Status[2]);
    }
}




uint8_t BATTERY_Get_DS_State(uint16_t p_u16PC1, uint8_t p_u8TypeBattery)
{
    if (p_u8TypeBattery == 1)
    {
        battery_get_DS_State_old(p_u16PC1, p_u8TypeBattery);
    }
    else if (p_u8TypeBattery == 2)
    {
        battery_get_DS_State_new(p_u16PC1, p_u8TypeBattery);
    }
    else
    {
        /*nothing to do or error*/
    }
}

uint8_t battery_get_DS_State_new(uint16_t p_u16PC1, uint8_t p_u8TypeBattery)
{
    uint32_t l_u32CurrentTime = get_ticks();
    static uint32_t l_su32ReactiveWDG = 0;
    static uint32_t l_su32CntTimeout = 0;
    static uint32_t l_su32CntOverheat = 0;
    static uint32_t l_su32CntFreeze = 0;

    /* TImeout pulse check*/
    if (l_u32CurrentTime - battery_u32StartTime_AnalogWDG < battery_cu32TimeoutPulse)
    {
        /* A pulse was received, OK*/
        if (battery_u32StartTime_AnalogWDG != battery_u32PulseTime_AnalogWDG)
        {
            l_su32CntTimeout = 0;
        }
    }
    else
    {
        /* Timeout error ! reset */
        BATTERY_DesactiveAnalogWatchdog();
        battery_u32StartTime_AnalogWDG = l_u32CurrentTime;
        battery_u32PulseTime_AnalogWDG = l_u32CurrentTime;
        if (l_su32CntTimeout > 50)
        {
            /* erreur ! state 3 maybe */
        }
        else
        {
            l_su32CntTimeout++;
        }
    }

    if( (l_u32CurrentTime - l_su32ReactiveWDG) > battery_cu32ReactivateWDG)
    {
        l_su32ReactiveWDG = l_u32CurrentTime;
        BATTERY_ActiveAnalogWatchdog();
    }

    /* normal range */
    if (p_u16PC1 > 1613 && p_u16PC1 < 2110)
    {
        /*normal stat*/
    }

    /* low temp ?*/
    if (p_u16PC1 > 2730)
    {
        if (l_su32CntFreeze > battery_cu32WaitError)
        {
            /* erreur ! state 0 maybe */
        }
        else
        {
            l_su32CntFreeze++;
        }
    }

    /* disconnected or overheat */
    if (p_u16PC1 < 248)
    {
        if (l_su32CntOverheat > battery_cu32WaitError)
        {
            /* erreur ! state 2 maybe */
        }
        else
        {
            l_su32CntOverheat++;
        }
    }
}

uint8_t battery_get_DS_State_old(uint16_t p_u16PC1, uint8_t p_u8TypeBattery)
{
    static uint32_t l_su32CntOverheat = 0;
    static uint32_t l_su32CntFreeze = 0;

    /* low temp?*/
    if (p_u16PC1 > 2730)
    {
        if (l_su32CntFreeze > battery_cu32WaitError)
        {
            /* erreur ! state 0 maybe */
        }
        else
        {
            l_su32CntFreeze++;
        }
    }
    /* disconnected or overheat */
    if (p_u16PC1 < 248)
    {
        if (l_su32CntOverheat > battery_cu32WaitError)
        {
            /* erreur ! state 2 maybe */
        }
        else
        {
            l_su32CntOverheat++;
        }
    }

    /* normal  use*/
    if (p_u16PC1 > 1489 && p_u16PC1 < 1985)
    {
        /*normal stat*/
        return 1;
    }
}

/*!
    \brief  battery_Set_Mode(uint32_t mode);
    all the logic to read the battery data on CS line
    \param[in]  p_emode
    \param[out] none
    \retval     return code 
*/
int32_t battery_Set_Mode(battery_mode_e p_eMode)
{   
    int32_t l_s32Return = -1;
    static battery_mode_e l_eCurrentMode = MODE_MAX;

    if(p_eMode < MODE_MAX)
    {
        if(p_eMode != l_eCurrentMode){
            timer_disable(TIMER2);
            switch (p_eMode)
            {
            case MODE_CS_CHARGE:
                battery_CSState = CS_STATE_INIT;
                battery_u32StartTime_CS = get_ticks();
                battery_u32StatusReceived = 0;
                BATTERY_DesactiveAnalogWatchdog();
                exti_interrupt_disable(EXTI_8);
                exti_interrupt_flag_clear(EXTI_8);
                exti_interrupt_enable(EXTI_8);
                /* desactivate DS */
                gpio_bit_reset(GPIOB, GPIO_PIN_6);
                /* activate CS */
                gpio_bit_set(GPIOB, GPIO_PIN_7);
                break;
            
            case MODE_DS:
                battery_u32NbPulse = 0;
                battery_CSState = CS_STATE_OFF;
                BATTERY_ActiveAnalogWatchdog();
                exti_interrupt_disable(EXTI_8);
                /* activate DS */
                gpio_bit_set(GPIOB, GPIO_PIN_6);
                /* desactivate CS */
                gpio_bit_reset(GPIOB, GPIO_PIN_7);
                break;

            case MODE_CS_IDLE:
            default:
                BATTERY_DesactiveAnalogWatchdog();
                exti_interrupt_disable(EXTI_8);
                battery_CSState = CS_STATE_OFF;
                /* desactivate DS */
                gpio_bit_reset(GPIOB, GPIO_PIN_6);
                /* activate CS */
                gpio_bit_set(GPIOB, GPIO_PIN_7);
                break;
            }
        }
        l_eCurrentMode = p_eMode;
        l_s32Return = 0; /* ok return*/
    }
    else
    {
        /* range error*/
        l_s32Return = -2;
    }
    return l_s32Return;
}
