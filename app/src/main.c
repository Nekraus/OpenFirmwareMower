#include "main.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "../include/microros_time.h"
#include "microros_transports.h"
#include <rmw_microros/rmw_microros.h>

#define BYTE_POOL_SIZE 9120

TX_THREAD adc_thread;
TX_THREAD io_thread;
TX_THREAD display_thread;
TX_THREAD motors_thread;
TX_THREAD battery_thread;
TX_THREAD imu_thread;

TX_BYTE_POOL byte_pool;
UCHAR memory_area[BYTE_POOL_SIZE];

rcl_publisher_t publisher;

#define AZURE_THREAD_STACK_SIZE 5000
#define AZURE_THREAD_PRIORITY 4

TX_THREAD azure_thread;
ULONG azure_thread_stack[AZURE_THREAD_STACK_SIZE / sizeof(ULONG)];

// void usart0_init(void);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/

void subscription_callback(const void *msgin)
{
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;

  if (msg->data == 0)
  {
    gpio_bit_set(GPIOF, GPIO_PIN_11);
  }
  else
  {
    gpio_bit_reset(GPIOF, GPIO_PIN_11);
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)last_call_time;
  (void)timer;

  static std_msgs__msg__Int32 msg = {0};

  if (RMW_RET_OK == rcl_publish(&publisher, &msg, NULL))
  {
    // printf("Sent: %ld\n", msg.data);
    msg.data++;
  }
  else
  {
    // printf("Failed to send\n");
  }
}

void microros_thread(ULONG parameter)
{

  rcl_ret_t res;
  // UINT status;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  // create init_options
  rclc_support_t support;
  res = rclc_support_init(&support, 0, NULL, &allocator);

  // create nodes
  rcl_node_t node;
  res = rclc_node_init_default(&node, "threadx_node", "", &support);

  // create publisher
  res = rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "threadx_publisher");

  // create subscriber
  rcl_subscription_t subscriber;
  res = rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "threadx_subscriber");

  // create timer,
  rcl_timer_t timer;
  res = rclc_timer_init_default2(
      &timer,
      &support,
      RCL_MS_TO_NS(10),
      timer_callback,
      true);

  // create executor
  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
  res = rclc_executor_init(&executor, &support.context, 2, &allocator);

  std_msgs__msg__Int32 msg = {0};
  res = rclc_executor_add_subscription(&executor, &subscriber, &msg, subscription_callback, ON_NEW_DATA);
  res = rclc_executor_add_timer(&executor, &timer);

  /* This thread simply sits in while-forever-sleep loop.  */
  while (1)
  {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    tx_thread_sleep((ULONG)0.1 * TX_TIMER_TICKS_PER_SECOND);
  }
  // Free resources.
  (void)!rcl_subscription_fini(&subscriber, &node);
  (void)!rcl_publisher_fini(&publisher, &node);
  (void)!rcl_node_fini(&node);
}

void tx_application_define(void *first_unused_memory)
{

  // // Create Azure thread
  // UINT status = tx_thread_create(&azure_thread,
  //                                "micro-ROS thread",
  //                                microros_thread,
  //                                0,
  //                                azure_thread_stack,
  //                                AZURE_THREAD_STACK_SIZE,
  //                                AZURE_THREAD_PRIORITY,
  //                                AZURE_THREAD_PRIORITY,
  //                                TX_NO_TIME_SLICE,
  //                                TX_AUTO_START);

  // if (status != TX_SUCCESS)
  // {
  //   // printf("Thread creation failed\r\n");
  //   tx_thread_sleep(5);
  // }
  CHAR *pointer = TX_NULL;

  /* Create a byte memory pool from which to allocate the thread stacks.  */
  tx_byte_pool_create(&byte_pool, "byte pool 0", memory_area, BYTE_POOL_SIZE);

  /* Put system definition stuff in here, e.g. thread creates and other assorted
     create information.  */

  /* Allocate the stack for thread 0.  */
  tx_byte_allocate(&byte_pool, (VOID **)&pointer, AZURE_THREAD_STACK_SIZE, TX_NO_WAIT);

  /* Create the main thread.  */
  tx_thread_create(&azure_thread, "microros_thread", microros_thread, 0,
                   pointer, AZURE_THREAD_STACK_SIZE,
                   4, 4, TX_NO_TIME_SLICE, TX_AUTO_START);

  // tx_thread_create(&adc_thread, "ADC",
  //                  ADC_App, 0, pointer, 1024,
  //                  3, 3, TX_NO_TIME_SLICE, TX_AUTO_START);

  // if (tx_byte_allocate(&byte_pool, (VOID **)&pointer,
  //                      1024, TX_NO_WAIT) != TX_SUCCESS)
  // {
  //   // return TX_POOL_ERROR;
  // }

  // tx_thread_create(&io_thread, "IO",
  //                  DIGITALIO_App, 0, pointer, 1024,
  //                  10, 10, TX_NO_TIME_SLICE, TX_AUTO_START);

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

  // if (tx_byte_allocate(&byte_pool, (VOID **)&pointer,
  //                      1024, TX_NO_WAIT) != TX_SUCCESS)
  // {
  //   // return TX_POOL_ERROR;
  // }

  // tx_thread_create(&battery_thread, "BATTERY",
  //                  BATTERY_App, 0, pointer, 1024,
  //                  3, 3, TX_NO_TIME_SLICE, TX_AUTO_START);

  // if (tx_byte_allocate(&byte_pool, (VOID **)&pointer,
  //                      1024, TX_NO_WAIT) != TX_SUCCESS)
  // {
  //   // return TX_POOL_ERROR;
  // }

  // tx_thread_create(&imu_thread, "IMU",
  //                  IMU_App, 0, pointer, 1024,
  //                  3, 3, TX_NO_TIME_SLICE, TX_AUTO_START);
}

void thread_sleepUntil(uint32_t *const previousWakeTime, const uint32_t timeIncrement)
{
  const uint32_t currentTime = tx_time_get();

  tx_thread_sleep(timeIncrement - (currentTime - *previousWakeTime));

  *previousWakeTime = *previousWakeTime + timeIncrement;
}

int main(void)
{

  // init controllable LED
  rcu_periph_clock_enable(RCU_GPIOF);
  gpio_init(GPIOF, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_11);

  systick_interval_set(TX_TIMER_TICKS_PER_SECOND);

  // Configure micro-ROS Serial Agent, using USART0 which is the only accessible UART on the mainboard.
  static serial_transport_args serial_comm_args = {
      .baud_rate = 115200U,
      .parity = USART_PM_NONE,
      .stop_bits = USART_STB_1BIT,
      .word_length = USART_WL_8BIT};

  microros_usart_init(USART0, serial_comm_args.baud_rate);

  rmw_uros_set_custom_transport(
      true,
      (void *)USART0,
      usart_it_transport_open,
      usart_it_transport_close,
      usart_it_transport_write,
      usart_it_transport_read);

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