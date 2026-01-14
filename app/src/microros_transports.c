/* Code taken in large majority from https://github.com/GigaDeviceSemiconductor/GD32H7-micro_ROS/blob/main/GD32H75E_Eval_FreeRTOS_MicorROS/Project/FreeRTOS_miocroROS/extra_sources/microros_transports/usart_it_transport.c */

#include <microros_transports.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

// --- micro-ROS Transports ---
#define UART_IT_BUFFER_SIZE 2048

#define UART_TRANS_TIMEOUT (ULONG)0.100 * TX_TIMER_TICKS_PER_SECOND // in number of ticks

static uint8_t it_buffer[UART_IT_BUFFER_SIZE];
static uint8_t it_data;
static size_t it_head = 0, it_tail = 0;

/* GD32 USART peripheral definition */
uint32_t gd32_usart = USART0;

bool usart_it_transport_open(struct uxrCustomTransport *transport)
{
  uint32_t usart_periph = gd32_usart;

  if (transport->args != NULL)
  {
    usart_periph = ((uint32_t)transport->args);
  }
  else
  {
  }

  /* clear interrupt flags */
  usart_interrupt_flag_clear(usart_periph, USART_INT_FLAG_RBNE);
  /* enable USART receive interrupt */
  usart_interrupt_enable(usart_periph, USART_INT_RBNE);

  return true;
}

bool usart_it_transport_close(struct uxrCustomTransport *transport)
{
  uint32_t usart_periph = gd32_usart;

  if (transport->args != NULL)
  {
    usart_periph = ((uint32_t)transport->args);
  }

  /* disable USART receive interrupt */
  usart_interrupt_disable(usart_periph, USART_INT_RBNE);

  return true;
}

size_t usart_it_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err)
{
  uint32_t usart_periph = gd32_usart;
  uint32_t timeout = 0;

  if (transport->args != NULL)
  {
    usart_periph = ((uint32_t)transport->args);
  }

  size_t written = 0;

  /* transmit data */
  for (written = 0; written < len; written++)
  {
    /* wait transmit buffer empty */
    while (RESET == usart_flag_get(usart_periph, USART_FLAG_TBE))
      ;

    /* transmit data */
    usart_data_transmit(usart_periph, buf[written]);
  }

  timeout = 0;
  /* wait transmit complete */
  while (RESET == usart_flag_get(usart_periph, USART_FLAG_TC) && timeout < UART_TRANS_TIMEOUT)
  {
    timeout++;
    tx_thread_sleep(1);
  }

  return written;
}

size_t usart_it_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
  size_t wrote = 0;

  while ((it_head != it_tail) && (wrote < len))
  {
    buf[wrote] = it_buffer[it_head];
    it_head = (it_head + 1) % UART_IT_BUFFER_SIZE;
    wrote++;
  }

  return wrote;
}

/*!
    \brief      this function handles USARTx exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void gd32_usart_irq_handler(uint32_t usart_periph)
{
  if (RESET != usart_interrupt_flag_get(usart_periph, USART_INT_FLAG_RBNE))
  {
    /* read received data */
    it_data = (uint8_t)usart_data_receive(usart_periph);

    /* calculate next write position */
    size_t next_tail = (it_tail + 1) % UART_IT_BUFFER_SIZE;

    /* check buffer overflow */
    if (next_tail != it_head)
    {
      it_buffer[it_tail] = it_data;
      it_tail = next_tail;
    }

    /* if buffer full, discard new data (can be modified as needed) */

    /* clear interrupt flag */
    usart_interrupt_flag_clear(usart_periph, USART_INT_FLAG_RBNE);
  }
}

/*!
    \brief      this function handles USART2 exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART0_IRQHandler(void)
{
  gd32_usart_irq_handler(USART0);
}

/*!
    \brief      this function handles USART2 exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART1_IRQHandler(void)
{
  gd32_usart_irq_handler(USART1);
}

/*!
    \brief      this function handles USART2 exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART2_IRQHandler(void)
{
  gd32_usart_irq_handler(USART2);
}

/*!
    \brief      this function initialize USART GPIO
    \param[in]  usart_periph: USARTx(x=0,1,2)
    \param[out] none
    \retval     none
*/
static void usart_gpio_init(uint32_t usart_periph)
{
  /* enable peripheral clock */
  switch (usart_periph)
  {
  case USART0:
    rcu_periph_clock_enable(RCU_GPIOA);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_MAX, GPIO_PIN_9);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_MAX, GPIO_PIN_10);
    break;
  case USART1:
    rcu_periph_clock_enable(RCU_GPIOA);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_MAX, GPIO_PIN_2);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_MAX, GPIO_PIN_3);
    break;
  case USART2:
    rcu_periph_clock_enable(RCU_GPIOB);
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
    break;
  default:
    break;
  }
}

/*!
    \brief      initialize micro-ROS USART
    \param[in]  usart_periph: USARTx(x=0,1,2,5), UARTx(x=3,4,6,7)
    \param[in]  baudrate: baudrate
    \param[out] none
    \retval     none
*/
void microros_usart_init(uint32_t usart_periph, uint32_t baudrate)
{
  /* enable peripheral clock */
  switch (usart_periph)
  {
  case USART0:
    rcu_periph_clock_enable(RCU_USART0);
    usart_gpio_init(USART0);
    nvic_irq_enable(USART0_IRQn, 2, 0);
    break;
  case USART1:
    rcu_periph_clock_enable(RCU_USART1);
    usart_gpio_init(USART1);
    nvic_irq_enable(USART1_IRQn, 2, 0);
    break;
  case USART2:
    rcu_periph_clock_enable(RCU_USART2);
    usart_gpio_init(USART2);
    nvic_irq_enable(USART2_IRQn, 2, 0);
    break;
  default:
    break;
  }

  /* USART configure */
  usart_deinit(usart_periph);
  usart_baudrate_set(usart_periph, baudrate);
  usart_word_length_set(usart_periph, USART_WL_8BIT);
  usart_stop_bit_set(usart_periph, USART_STB_1BIT);
  usart_parity_config(usart_periph, USART_PM_NONE);
  usart_hardware_flow_rts_config(usart_periph, USART_RTS_DISABLE);
  usart_hardware_flow_cts_config(usart_periph, USART_CTS_DISABLE);
  usart_receive_config(usart_periph, USART_RECEIVE_ENABLE);
  usart_transmit_config(usart_periph, USART_TRANSMIT_ENABLE);

  /* USART enable */
  usart_enable(usart_periph);

  /* clear interrupt flags */
  usart_interrupt_flag_clear(usart_periph, USART_INT_FLAG_RBNE);
  usart_interrupt_enable(usart_periph, USART_INT_RBNE);
}

#endif // RMW_UXRCE_TRANSPORT_CUSTOM