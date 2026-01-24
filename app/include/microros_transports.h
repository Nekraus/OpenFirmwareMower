#ifndef MICROROS_TRANSPORTS__H
#define MICROROS_TRANSPORTS__H

#include <rmw_microxrcedds_c/config.h>
#include <uxr/client/transport.h>
#include "gd32f30x.h"
#include "gd32f30x_it.h"
#include "tx_api.h"

typedef struct serial_transport_args
{
  uint32_t baud_rate;
  uint32_t word_length;
  uint32_t parity;
  uint32_t stop_bits;
} serial_transport_args;

#ifdef __cplusplus
extern "C"
{
#endif

  bool usart_it_transport_open(struct uxrCustomTransport *transport);
  bool usart_it_transport_close(struct uxrCustomTransport *transport);
  size_t usart_it_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
  size_t usart_it_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);
  void microros_usart_init(uint32_t usart_periph, uint32_t baudrate);

#ifdef __cplusplus
}
#endif

#endif // MICROROS_TRANSPORTS__H