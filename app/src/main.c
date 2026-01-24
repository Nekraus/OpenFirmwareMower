#include "main.h"

#include "../include/digitalIO.h"
#include "../include/display.h"
#include "../include/eeprom.h"
#include "../include/imu.h"
#include "../include/motors.h"
#include "../include/battery.h"
#include "../include/adc.h"
#include "../include/microros.h"

#define BYTE_POOL_SIZE 9120
#define AZURE_THREAD_STACK_SIZE 3000
#define AZURE_THREAD_PRIORITY 4

TX_THREAD adc_thread;
TX_THREAD io_thread;
TX_THREAD display_thread;
TX_THREAD motors_thread;
TX_THREAD battery_thread;
TX_THREAD imu_thread;
TX_THREAD microros_thread;

TX_BYTE_POOL byte_pool;
UCHAR memory_area[BYTE_POOL_SIZE];

ULONG azure_thread_stack[AZURE_THREAD_STACK_SIZE / sizeof(ULONG)];

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/

void tx_application_define(void *first_unused_memory)
{

  BATTERY_init();
  // MOTORS_Init();
  // DISPLAY_Init();

  CHAR *pointer = TX_NULL;

  /* Create a byte memory pool from which to allocate the thread stacks.  */
  tx_byte_pool_create(&byte_pool, "byte pool 0", memory_area, BYTE_POOL_SIZE);

  /* Put system definition stuff in here, e.g. thread creates and other assorted
     create information.  */

  /* Allocate the stack for thread 0.  */
  tx_byte_allocate(&byte_pool, (VOID **)&pointer, AZURE_THREAD_STACK_SIZE, TX_NO_WAIT);

  /* Create the main thread.  */
  tx_thread_create(&microros_thread, "microros_thread", MICROROS_App, 0,
                   pointer, AZURE_THREAD_STACK_SIZE,
                   1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);

  if (tx_byte_allocate(&byte_pool, (VOID **)&pointer,
                       1024, TX_NO_WAIT) != TX_SUCCESS)
  {
    // return TX_POOL_ERROR;
  }
  tx_thread_create(&adc_thread, "ADC",
                   ADC_App, 0, pointer, 1024,
                   3, 3, TX_NO_TIME_SLICE, TX_AUTO_START);

  if (tx_byte_allocate(&byte_pool, (VOID **)&pointer,
                       1024, TX_NO_WAIT) != TX_SUCCESS)
  {
    // return TX_POOL_ERROR;
  }

  tx_thread_create(&io_thread, "IO",
                   DIGITALIO_App, 0, pointer, 1024,
                   10, 10, TX_NO_TIME_SLICE, TX_AUTO_START);

  // if (tx_byte_allocate(&byte_pool, (VOID **)&pointer,
  //                      1024, TX_NO_WAIT) != TX_SUCCESS)
  // {
  //   // return TX_POOL_ERROR;
  // }

  // tx_thread_create(&display_thread, "DISPLAY",
  //                  DISPLAY_App, 0, pointer, 1024,
  //                  3, 3, TX_NO_TIME_SLICE, TX_AUTO_START);

  // if (tx_byte_allocate(&byte_pool, (VOID **)&pointer,
  //                      1024, TX_NO_WAIT) != TX_SUCCESS)
  // {
  //   // return TX_POOL_ERROR;
  // }

  // tx_thread_create(&motors_thread, "MOTORS",
  //                  MOTORS_App, 0, pointer, 1024,
  //                  3, 3, TX_NO_TIME_SLICE, TX_AUTO_START);

  if (tx_byte_allocate(&byte_pool, (VOID **)&pointer,
                       1024, TX_NO_WAIT) != TX_SUCCESS)
  {
    // return TX_POOL_ERROR;
  }

  tx_thread_create(&battery_thread, "BATTERY",
                   BATTERY_App, 0, pointer, 1024,
                   3, 3, TX_NO_TIME_SLICE, TX_AUTO_START);

  if (tx_byte_allocate(&byte_pool, (VOID **)&pointer,
                       1024, TX_NO_WAIT) != TX_SUCCESS)
  {
    // return TX_POOL_ERROR;
  }

  tx_thread_create(&imu_thread, "IMU",
                   IMU_App, 1, pointer, 1024,
                   4, 4, TX_NO_TIME_SLICE, TX_AUTO_START);
}

void thread_sleepUntil(uint32_t *const previousWakeTime, const uint32_t timeIncrement)
{
  const uint32_t currentTime = tx_time_get();

  tx_thread_sleep(timeIncrement - (currentTime - *previousWakeTime));

  *previousWakeTime = *previousWakeTime + timeIncrement;
}

int main(void)
{

  systick_interval_set(TX_TIMER_TICKS_PER_SECOND);

  /* Enter the ThreadX kernel. */
  tx_kernel_enter();
}

// /* retarget the gcc's C library printf function to the USART */
// int _write(int file, char *data, int len)
// {
//   if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
//   {
//     errno = EBADF;
//     return -1;
//   }

//   // arbitrary timeout 1000
//   for (int i = 0; i < len; i++)
//   {
//     usart_data_transmit(USART1, (uint8_t)data[i]);
//     while (RESET == usart_flag_get(USART1, USART_FLAG_TBE))
//       ;
//   }

//   // return # of bytes written - as best we can tell
//   return len;
// }