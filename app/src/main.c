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

#include "../include/main.h"
#include "../include/digitalIO.h"
#include "../include/display.h"
#include "../include/eeprom.h"
#include "../include/imu.h"
#include "../include/motors.h"
#include "../include/battery.h"
#include "../include/adc.h"

#include  <errno.h>
#include  <sys/unistd.h> 

TX_THREAD adc_thread;

void usart0_init(void);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    /* init usart0 for printf */
    usart0_init();
    /* Enter the ThreadX kernel. */
    tx_kernel_enter( );
}

void tx_application_define(void *first_unused_memory)
{
    TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;
    CHAR *pointer;


    if (tx_byte_allocate(byte_pool, (VOID**) &pointer,
                        1024, TX_NO_WAIT) != TX_SUCCESS)
    {
        //return TX_POOL_ERROR;
    }

    tx_thread_create(&adc_thread, "ADC",
    ADC_App, 0, pointer, 1024,
    3, 3, TX_NO_TIME_SLICE, TX_AUTO_START);

    
    if (tx_byte_allocate(byte_pool, (VOID**) &pointer,
                        1024, TX_NO_WAIT) != TX_SUCCESS)
    {
        //return TX_POOL_ERROR;
    }
}


void thread_sleepUntil(uint32_t * const previousWakeTime, const uint32_t timeIncrement)
{
    const uint32_t currentTime = tx_time_get();

    tx_thread_sleep(timeIncrement - (currentTime - *previousWakeTime));

    *previousWakeTime = *previousWakeTime + timeIncrement;
}

void usart0_init(void){
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
})

/* retarget the gcc's C library printf function to the USART */
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
