/*
 * BufferedSerial.h
 *
 *  Created on: 10/07/2025
 *      Author: nekraus
 */

#ifndef BUFFEREDSERIAL_H_
#define BUFFEREDSERIAL_H_

#include <string.h>
#include "main.h"

#define RX_BUF_SIZE 512
#define TX_BUF_SIZE 512

class BufferedSerial {
 private:
  // UART:
  uint32_t uart;
  uint32_t dma_rx;
  dma_channel_enum dma_rx_channel;
  uint32_t dma_tx;
  dma_channel_enum dma_tx_channel;

  // Buffers:
  static constexpr uint16_t rx_buf_mask = RX_BUF_SIZE - 1;
  static constexpr uint16_t tx_buf_mask = TX_BUF_SIZE - 1;
  uint8_t rx_buf[RX_BUF_SIZE];
  uint8_t tx_buf[TX_BUF_SIZE];

  // Indexes:
  uint16_t rx_tail = 0;
  uint16_t tx_head = 0;
  uint16_t tx_tail = 0;
  uint16_t tx_end = TX_BUF_SIZE;

 public:
  BufferedSerial(uint32_t uart_);
  BufferedSerial();


  void init(void);
  int read(void);
  void write(const uint8_t *const c, const int length);
  void flush_tx_buffer();

  void tx_cplt_callback(void);
  void reset_rx_buffer(void);
};

#endif /* BUFFEREDSERIAL_H_ */
