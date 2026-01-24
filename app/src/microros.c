#include "microros.h"

rcl_publisher_t imu_publisher;
rcl_publisher_t battery_publisher;
bool microros_started = false;

static char imu_topic[] = "imu/data_raw";
static char battery_topic[] = "power";

void MICROROS_App(ULONG parameter)
{

  // Configure micro-ROS Serial Agent, using USART0 which is the only accessible UART on the mainboard.
  static serial_transport_args serial_comm_args = {
      .baud_rate = 921600U,
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

  rcl_ret_t res;
  // UINT status;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  // create init_options
  rclc_support_t support;
  res = rclc_support_init(&support, 0, NULL, &allocator);

  // create nodes
  rcl_node_t mower_node;
  res = rclc_node_init_default(&mower_node, "mower_node", "", &support);

  // sync epoch
  rmw_ret_t epoch_sync = RMW_RET_ERROR;

  int retries = 10;
  do
  {
    epoch_sync = rmw_uros_sync_session(1000);
  } while ((epoch_sync != RMW_RET_OK) && (retries-- >= 0));

  // create IMU publisher
  res = rclc_publisher_init_best_effort(
      &imu_publisher,
      &mower_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      imu_topic);

  // create Battery publisher
  res = rclc_publisher_init_best_effort(
      &battery_publisher,
      &mower_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
      battery_topic);

  // // create subscriber
  // rcl_subscription_t subscriber;
  // res = rclc_subscription_init_default(
  //     &subscriber,
  //     &mower_node,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  //     "threadx_subscriber");

  // create timer,
  // rcl_timer_t timer;
  // res = rclc_timer_init_default2(
  //     &timer,
  //     &support,
  //     RCL_MS_TO_NS(10),
  //     timer_callback,
  //     true);

  // create executor
  // rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
  // res = rclc_executor_init(&executor, &support.context, 2, &allocator);

  // res = rclc_executor_add_subscription(&executor, &subscriber, &msg, subscription_callback, ON_NEW_DATA);
  // res = rclc_executor_add_timer(&executor, &timer);

  if (res == RCL_RET_OK)
  {
    microros_started = true;

    /* This thread simply sits in while-forever-sleep loop.  */
    while (1)
    {
      // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      tx_thread_sleep((ULONG)(0.1 * TX_TIMER_TICKS_PER_SECOND));
      // tx_thread_sleep(1);
    }
  }
  // Free resources.
  // (void)!rcl_subscription_fini(&subscriber, &mower_node);
  (void)!rcl_publisher_fini(&imu_publisher, &mower_node);
  (void)!rcl_publisher_fini(&battery_publisher, &mower_node);
  (void)!rcl_node_fini(&mower_node);
}