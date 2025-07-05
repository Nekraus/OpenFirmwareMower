/*
 * battery.h 
 *
 *  Created on: 03/07/2025 
 *      Author: Bruno Lecornu 
 */

#ifndef BATTERY_H
#define BATTERY_H
/* +-----------------------------------------------------------------------+ */
/* |                        CONSTANTES / MACROS                            | */
/* +-----------------------------------------------------------------------+ */

/* +-----------------------------------------------------------------------+ */
/* |                            TYPEDEFS                                   | */
/* +-----------------------------------------------------------------------+ */

/* +-----------------------------------------------------------------------+ */
/* |                         GLOBAL VARIABLES                              | */
/* +-----------------------------------------------------------------------+ */

/* +-----------------------------------------------------------------------+ */
/* |                         PUBLIC FUNCTIONS                              | */
/* +-----------------------------------------------------------------------+ */

/*!
    \brief      BATTERY init function
    Init the ADC to count the pulse on DS
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BATTERY_init(void);

/*!
    \brief      BATTERY_ActiveAnalogWatchdog
    Activate the analog watchdog on ADC0 on PC1
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BATTERY_ActiveAnalogWatchdog(void);

/*!
    \brief      BATTERY_DesactiveAnalogWatchdog
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BATTERY_DesactiveAnalogWatchdog(void);

/*!
    \brief      BATTERY_AnalogWatchdogIRQ
    Need to be set in the ADC IRQ, function to count pulses on DS
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BATTERY_AnalogWatchdogIRQ(void);

/*!
    \brief              BATTERY_ExtlineIRQ();
    Need to be set in the Extline IRQ, function to read the UART stye data on CS
    \param[in]  none
    \param[out] none
    \retval     none
*/

void BATTERY_ExtlineIRQ(void);
/*!
    \brief BATTERY_ExtlineIRQ();
    Need to be set in the Timer IRQ, function to read the UART stye data on CS
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BATTERY_TimerIRQ(void);

/*!
    \brief              BATTERY_App();
    all the logic to read the battery data on CS line
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BATTERY_App(void);

uint8_t BATTERY_Monitoring(int32_t p_u32Voltage, int32_t p_u32Current, uint8_t p_u8Type, uint16_t p_u16Status);

uint8_t BATTERY_Get_DS_State(uint16_t p_u16PC1, uint8_t p_u8TypeBattery);

#endif /* BATTERY_H*/