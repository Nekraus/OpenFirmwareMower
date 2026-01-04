#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <microros_transports.h>

// #define UART_TXD (CONFIG_MICROROS_UART_TXD)
// #define UART_RXD (CONFIG_MICROROS_UART_RXD)
// #define UART_RTS (CONFIG_MICROROS_UART_RTS)
// #define UART_CTS (CONFIG_MICROROS_UART_CTS)

// --- micro-ROS Transports ---

bool microros_transport_open(struct uxrCustomTransport *transport)
{

  serial_transport_args *args = (serial_transport_args *)transport->args;

  /* enable GPIO clock */
  rcu_periph_clock_enable(RCU_GPIOA);

  /* enable UART clock */
  rcu_periph_clock_enable(RCU_USART0);

  /* connect port to UARTx_Tx */
  gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

  /* connect port to UARTx_Rx */
  gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

  /* UART configure */
  usart_deinit(USART0);

  usart_baudrate_set(USART0, args->baud_rate);
  usart_word_length_set(USART0, args->word_length);
  usart_parity_config(USART0, args->parity);
  usart_stop_bit_set(USART0, args->stop_bits);
  usart_receive_config(USART0, USART_RECEIVE_ENABLE);
  usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);

  usart_enable(USART0);

  return true;
}

bool microros_transport_close(struct uxrCustomTransport *transport)
{
  usart_disable(USART0);

  return true;
}

size_t microros_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err)
{

  for (int i = 0; i < len; i++)
  {
    usart_data_transmit(USART0, (uint8_t)buf[i]);
    while (RESET == usart_flag_get(USART0, USART_FLAG_TBE))
      ;
  }

  // return # of bytes written - as best we can tell
  return len;
}

size_t microros_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
  unsigned int read_counter = 0;
  usart_data_receive(USART0);
  int init_time = tx_time_get();

  while (read_counter < len)
  {
    /* wait until end of transmit */
    while (RESET == usart_flag_get(USART0, USART_FLAG_TC))
    {
    }
    /* wait the byte is entirely received by USART0 */
    while (RESET == usart_flag_get(USART0, USART_FLAG_RBNE) && ((int)tx_time_get() - init_time) < timeout * (int)TX_TIMER_TICKS_PER_SECOND / 1000)
      ;

    if (((int)tx_time_get() - init_time) < timeout * (int)TX_TIMER_TICKS_PER_SECOND / 1000)
    {
      /* store the received byte in the rxbuffer1 */
      buf[read_counter++] = usart_data_receive(USART0);
    }
    else
    {
      return 0;
    }
  }

  return read_counter;
}