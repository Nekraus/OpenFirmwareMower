#ifndef MICROROS_TRANSPORTS__H
#define MICROROS_TRANSPORTS__H

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

  bool microros_transport_open(struct uxrCustomTransport *transport);
  bool microros_transport_close(struct uxrCustomTransport *transport);
  size_t microros_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *error);
  size_t microros_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

#ifdef __cplusplus
}
#endif

#endif // MICROROS_TRANSPORTS__H