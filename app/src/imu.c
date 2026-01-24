/*
 * imu.c
 *
 *  Created on: 18/12/2024
 *      Author: Bruno Lecornu
 *  Changelog:
 *   - 2026-01-19: Christian-Nils, added micro-ros messages
 */

/* +-----------------------------------------------------------------------+ */
/* |                               HEADER                                  | */
/* +-----------------------------------------------------------------------+ */
#include "sensor_msgs/msg/imu.h"
#include "tx_api.h"
#include "imu.h"
/* +-----------------------------------------------------------------------+ */
/* |                            TYPEDEFS                                   | */
/* +-----------------------------------------------------------------------+ */

/* +-----------------------------------------------------------------------+ */
/* |                        CONSTANTES / MACROS                            | */
/* +-----------------------------------------------------------------------+ */

/* +-----------------------------------------------------------------------+ */
/* |                         GLOBAL VARIABLES                              | */
/* +-----------------------------------------------------------------------+ */
extern rcl_publisher_t imu_publisher;
extern bool microros_started;

/* +-----------------------------------------------------------------------+ */
/* |                         LOCAL VARIABLES                               | */
/* +-----------------------------------------------------------------------+ */
#define IMU_SAMPLING_RATE 50UL
#define IMU_SAMPLE_PERIOD_TICKS (uint32_t)(TX_TIMER_TICKS_PER_SECOND / IMU_SAMPLING_RATE)
uint32_t previous_imu_tick;
float temp_f = 0.0;
/* +-----------------------------------------------------------------------+ */
/* |                         PUBLIC FUNCTIONS                              | */
/* +-----------------------------------------------------------------------+ */
void IMU_App(ULONG thread_input)
{
  imu_init_hard();
  imu_init_component();

  // wait that microros communication is up and running
  while (!microros_started)
  {
    tx_thread_sleep((ULONG)(1 * TX_TIMER_TICKS_PER_SECOND));
  }

  imu_data_t imu_data;
  static sensor_msgs__msg__Imu imu_msg = {};

  imu_msg.header.frame_id.data = "imu";
  imu_msg.header.frame_id.size = 3;
  previous_imu_tick = tx_time_get();
  imu_msg.angular_velocity_covariance[0] = -1;
  imu_msg.linear_acceleration_covariance[0] = -1;
  imu_msg.orientation_covariance[0] = -1;
  imu_msg.orientation.w = 1;
  while (1)
  {
    imu_read_data(&imu_data);
    int64_t ns = rmw_uros_epoch_nanos();
    imu_msg.header.stamp.sec = ns / 1000000000;
    imu_msg.header.stamp.nanosec = ns % 1000000000;
    imu_msg.linear_acceleration.x = imu_data.ax;
    imu_msg.linear_acceleration.y = imu_data.ay;
    imu_msg.linear_acceleration.z = imu_data.az;
    imu_msg.angular_velocity.x = imu_data.gx;
    imu_msg.angular_velocity.y = imu_data.gy;
    imu_msg.angular_velocity.z = imu_data.gz;

    rcl_publish(&imu_publisher, &imu_msg, NULL);
    thread_sleepUntil(&previous_imu_tick, IMU_SAMPLE_PERIOD_TICKS);
    previous_imu_tick = tx_time_get();
  }
}

/* +-----------------------------------------------------------------------+ */
/* |                           LOCAL FUNCTIONS                             | */
/* +-----------------------------------------------------------------------+ */
void imu_init_hard(void)
{
  spi_parameter_struct spi_init_struct;

  rcu_periph_clock_enable(RCU_AF);
  rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_GPIOF);
  rcu_periph_clock_enable(RCU_SPI2);

  gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);

  gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_MAX, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
  gpio_init(GPIOF, GPIO_MODE_OUT_PP, GPIO_OSPEED_MAX, GPIO_PIN_15);

  /* deinitilize SPI and the parameters */
  spi_i2s_deinit(SPI2);
  spi_struct_para_init(&spi_init_struct);

  /* configure SPI2 parameter */
  spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
  spi_init_struct.device_mode = SPI_MASTER;
  spi_init_struct.frame_size = SPI_FRAMESIZE_8BIT;
  spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
  spi_init_struct.nss = SPI_NSS_SOFT;
  spi_init_struct.prescale = SPI_PSC_64;
  spi_init_struct.endian = SPI_ENDIAN_MSB;
  spi_init(SPI2, &spi_init_struct);
  spi_enable(SPI2);
}

/* hard to said what is configured as we don't know the chip */
void imu_init_component(void)
{
  uint8_t cmd[] = {0xc0, 0xd3, 0xc2, 0xc0, 0xd3, 0xc2, 0xd3, 0xd5, 0xd3, 0xd5, 0xd4, 0xd4, 0xbb, 0xb9, 0xba, 0xbb, 0xb9, 0xba};
  uint8_t data[] = {0x38, 0x16, 0x10, 0x3e, 0x12, 0x20, 0x11, 0x02, 0x12, 0x00, 0x08, 0x00, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00};
  uint8_t tmp;
  uint32_t index = 0;

  imu_read(0x0f, 1, &tmp);
  imu_read(0xdd, 1, &tmp);
  /* sleep 50ms*/
  tx_thread_sleep((ULONG)(0.050 * TX_TIMER_TICKS_PER_SECOND));
  imu_read(0xdd, 1, &tmp);
  /* sleep 50ms*/
  tx_thread_sleep((ULONG)(0.050 * TX_TIMER_TICKS_PER_SECOND));
  imu_write(cmd[index], 1, &data[index]);
  index++;
  imu_write(cmd[index], 1, &data[index]);
  index++;
  imu_write(cmd[index], 1, &data[index]);
  index++;
  /* sleep 500ms*/
  tx_thread_sleep((ULONG)(0.5 * TX_TIMER_TICKS_PER_SECOND));
  imu_write(cmd[index], 1, &data[index]);
  index++;
  imu_write(cmd[index], 1, &data[index]);
  index++;
  imu_write(cmd[index], 1, &data[index]);
  index++;
  /* sleep 100ms*/
  tx_thread_sleep((ULONG)(0.1 * TX_TIMER_TICKS_PER_SECOND));
  imu_write(cmd[index], 1, &data[index]);
  index++;
  imu_write(cmd[index], 1, &data[index]);
  index++;
  /* sleep 1ms*/
  tx_thread_sleep((ULONG)(0.001 * TX_TIMER_TICKS_PER_SECOND));
  imu_write(cmd[index], 1, &data[index]);
  index++;
  /* sleep 1ms*/
  tx_thread_sleep((ULONG)(0.001 * TX_TIMER_TICKS_PER_SECOND));
  imu_write(cmd[index], 1, &data[index]);
  index++;
  /* sleep 50ms*/
  tx_thread_sleep((ULONG)(0.050 * TX_TIMER_TICKS_PER_SECOND));
  imu_write(cmd[index], 1, &data[index]);
  index++;
  /* sleep 10ms*/
  tx_thread_sleep((ULONG)(0.010 * TX_TIMER_TICKS_PER_SECOND));
  imu_write(cmd[index], 1, &data[index]);
  index++;
  /* sleep 1ms*/
  tx_thread_sleep((ULONG)(0.001 * TX_TIMER_TICKS_PER_SECOND));
  imu_write(cmd[index], 1, &data[index]);
  index++;
  /* sleep 10ms*/
  tx_thread_sleep((ULONG)(0.010 * TX_TIMER_TICKS_PER_SECOND));
  imu_write(cmd[index], 1, &data[index]);
  index++;
  /* sleep 10ms*/
  tx_thread_sleep((ULONG)(0.010 * TX_TIMER_TICKS_PER_SECOND));
  imu_write(cmd[index], 1, &data[index]);
  index++;
  /* sleep 10ms*/
  tx_thread_sleep((ULONG)(0.010 * TX_TIMER_TICKS_PER_SECOND));
  imu_write(cmd[index], 1, &data[index]);
  index++;
  /* sleep 10ms*/
  tx_thread_sleep((ULONG)(0.010 * TX_TIMER_TICKS_PER_SECOND));
  imu_write(cmd[index], 1, &data[index]);
  index++;
  /* sleep 10ms*/
  tx_thread_sleep((ULONG)(0.010 * TX_TIMER_TICKS_PER_SECOND));
  imu_write(cmd[index], 1, &data[index]);
  index++;
  /* sleep 10ms*/
  tx_thread_sleep((ULONG)(0.010 * TX_TIMER_TICKS_PER_SECOND));

  imu_read(0x22, 1, &tmp);
  tmp = (tmp & 0xff) | 1;
  imu_write(0x22, 1, &tmp);
  tmp = 0x03;
  imu_write(0x23, 1, &tmp);
  tmp = 0x04;
  imu_write(0x25, 1, &tmp);
  imu_read(0x26, 1, &tmp);
  tmp = (tmp & 0x17) + 0x20;
  imu_write(0x26, 1, &tmp);

  imu_read(0x28, 1, &tmp);
  tmp = (tmp & 0xff) | 1;
  imu_write(0x28, 1, &tmp);
  tmp = 0x03;
  imu_write(0x29, 1, &tmp);
  tmp = 0x03;
  imu_write(0x8f, 1, &tmp);
  tmp = 0x03;
  imu_write(0x9f, 1, &tmp);
  tmp = 0x03;
  imu_write(0xaf, 1, &tmp);
  imu_read(0x2b, 1, &tmp);
  tmp = 0x0c | (tmp & 0xe3);
  imu_write(0x28, 1, &tmp);

  imu_read(0x20, 1, &tmp);
  tmp = 0x30 | 0x80 | (tmp & 0x4f);
  imu_write(0x20, 1, &tmp);

  imu_read(0xdf, 1, &tmp);
  if (tmp == 0x61) // Chip ID?
  {
    /* save parameter*/
  }

  // volatile int16_t imu_data[6];
  volatile int16_t temp;
  volatile uint16_t temp1;
  volatile uint16_t temp2;

  // int16_t *p_tmp;
  imu_read(0x20, 2, &tmp);
  temp1 = ((tmp & 0x000F) << 8);
  temp1 = temp1 + ((tmp & 0xFF00) >> 8);
  imu_read(0x0c, 2, &tmp);
  temp2 = ((tmp & 0x0FFF));
  temp = temp2 - temp1;

  temp_f = (float)temp;
  temp_f /= 16.0f;
  temp_f += 25.0f;
  // while (1)
  // {
  //   /* read acc and gyro*/
  //   imu_read(0x00, 12, (uint8_t *)imu_data);
  //   /*acc div by 8192
  //   div by calibration data ?
  //   x9.8
  //   so m/s2? */
  //   p_tmp = (int16_t *)&imu_data[0];
  //   acc_x = (float)*p_tmp / 8192.0f * 9.8f;
  //   p_tmp = (int16_t *)&imu_data[1];
  //   acc_y = (float)*p_tmp / 8192.0f * 9.8f;
  //   p_tmp = (int16_t *)&imu_data[2];
  //   acc_z = (float)*p_tmp / 8192.0f * 9.8f;
  //   /* gyro  div by 131.1
  //   multi by Pi
  //   div by 180
  //   so °/s ?*/
  //   p_tmp = (int16_t *)&imu_data[3];
  //   gyro_x = (float)*p_tmp / 131.1f * 3.14f / 180.0f;
  //   p_tmp = (int16_t *)&imu_data[4];
  //   gyro_y = (float)*p_tmp / 131.1f * 3.14f / 180.0f;
  //   p_tmp = (int16_t *)&imu_data[5];
  //   gyro_z = (float)*p_tmp / 131.1f * 3.14f / 180.0f;
  //   tx_thread_sleep((ULONG)0.1 * TX_TIMER_TICKS_PER_SECOND);
  // }
}

void imu_read_data(imu_data_t *data)
{
  int16_t *p_tmp;
  volatile int16_t imu_data[6];
  /* read acc and gyro*/
  imu_read(0x00, 12, (uint8_t *)&imu_data);
  /*acc div by 8192
  div by calibration data ?
  x9.8
  so m/s2? */
  p_tmp = (int16_t *)&imu_data[0];
  data->ax = (double)*p_tmp / 8192.0 * 9.8;
  p_tmp = (int16_t *)&imu_data[1];
  data->ay = (double)*p_tmp / 8192.0 * 9.8;
  p_tmp = (int16_t *)&imu_data[2];
  data->az = (double)*p_tmp / 8192.0 * 9.8;
  /* gyro  div by 131.1
  multi by Pi
  div by 180
  so °/s ?*/
  p_tmp = (int16_t *)&imu_data[3];
  data->gx = (double)*p_tmp / 131.1 * 3.14 / 180.0;
  p_tmp = (int16_t *)&imu_data[4];
  data->gy = (double)*p_tmp / 131.1 * 3.14 / 180.0;
  p_tmp = (int16_t *)&imu_data[5];
  data->gz = (double)*p_tmp / 131.1 * 3.14 / 180.0;
}

void imu_read(uint8_t p_u8Cmd, uint32_t p_u32Size, uint8_t *p_pu8Data)
{
  uint8_t tmp = (0x7f < p_u8Cmd);
  /* set the current bank address*/
  imu_cmd_writeBytes(0x7f, 1, &tmp);
  /* read the data*/
  imu_cmd_readBytes(p_u8Cmd | 0x80, p_u32Size, p_pu8Data);
}

void imu_write(uint8_t p_u8Cmd, uint32_t p_u32Size, uint8_t *p_pu8Data)
{
  uint8_t tmp = (0x7f < p_u8Cmd);
  /* set the current bank address*/
  imu_cmd_writeBytes(0x7f, 1, &tmp);
  /* write the data*/
  imu_cmd_writeBytes(p_u8Cmd & 0x7f, p_u32Size, p_pu8Data);
}

void imu_cmd_readBytes(uint8_t p_u8Cmd, uint32_t p_u32Size, uint8_t *p_pu8Data)
{
  gpio_bit_reset(GPIOF, GPIO_PIN_15);
  spi2_readwrite(p_u8Cmd);
  for (uint32_t i = 0; i < p_u32Size; i++)
  {
    *(p_pu8Data + i) = spi2_readwrite(0xff);
  }
  gpio_bit_set(GPIOF, GPIO_PIN_15);
}

void imu_cmd_writeBytes(uint8_t p_u8Cmd, uint32_t p_u32Size, uint8_t *p_pu8Data)
{
  gpio_bit_reset(GPIOF, GPIO_PIN_15);
  spi2_readwrite(p_u8Cmd);
  for (uint32_t i = 0; i < p_u32Size; i++)
  {
    spi2_readwrite(*(p_pu8Data + i));
  }
  gpio_bit_set(GPIOF, GPIO_PIN_15);
}

uint8_t spi2_readwrite(uint8_t byte)
{
  uint32_t l_u32cnt;
  uint8_t l_u8Return;

  l_u32cnt = 50000;
  do
  {
    l_u32cnt--;
  } while (l_u32cnt != 0 && (RESET == spi_i2s_flag_get(SPI2, SPI_FLAG_TBE)));
  if (l_u32cnt < 1)
  {
    l_u8Return = 0xff;
  }
  else
  {
    spi_i2s_data_transmit(SPI2, byte);
    l_u32cnt = 50000;
    do
    {
      l_u32cnt--;
    } while (l_u32cnt != 0 && (RESET == spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE)));
    if (l_u32cnt < 1)
    {
      l_u8Return = 0xff;
    }
    else
    {
      l_u8Return = spi_i2s_data_receive(SPI2);
    }
  }
  return l_u8Return;
}