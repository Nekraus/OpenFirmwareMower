#ifndef MICROROS_H_
#define MICROROS_H_

#include "microros_time.h"
#include "microros_transports.h"
// #include "microros_transports_dma.h"
#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/battery_state.h>
// #include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

void subscription_callback(const void *msgin);
void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void MICROROS_App(ULONG parameter);

#endif /* MICROROS_H_ */