/*
 * BufferedSerial.cpp
 *
 *  Created on: 10/07/2025
 *      Author: nekraus
 */

#include "BufferedSerial.hpp"

// Create Serial Buffer with UART2:
BufferedSerial buff_serial();

// Constructor:
BufferedSerial::BufferedSerial(uint32_t uart_)
  : uart(uart_) {}

// Init:
void BufferedSerial::init(void) {
  switch (uart)
  {
  case USART0:
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART0);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    dma_rx = DMA0;
    dma_rx_channel = DMA_CH4
    dma_tx = DMA0;
    dma_tx_channel = DMA_CH3
    break;
  case USART1:
    /* code */
    break;
  case USART2:
    /* code */
    break;
  case USART3:
    /* code */
    break;
  case USART4:
    /* code */
    break;
  
  default:
    break;
  }

  usart_deinit(uart);
  usart_baudrate_set(uart, 115200U);
  usart_receive_config(uart, USART_RECEIVE_ENABLE);
  usart_transmit_config(uart, USART_TRANSMIT_ENABLE);
  usart_enable(uart);

  dma_deinit(dma_tx, dma_tx_channel);
  dma_struct_para_init(&dma_init_struct);
  dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
  dma_init_struct.memory_addr = (uint32_t)tx_buf;
  dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
  dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
  dma_init_struct.number = 0;
  dma_init_struct.periph_addr = USART_DATA(uart);
  dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
  dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
  dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
  dma_init(dma_tx, dma_tx_channel, &dma_init_struct);

  dma_circulation_disable(dma_tx, dma_tx_channel);   
  dma_channel_enable(dma_tx, dma_tx_channel);

  dma_deinit(dma_rx, dma_rx_channel);
  dma_struct_para_init(&dma_init_struct);
  dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
  dma_init_struct.memory_addr = (uint32_t)rx_buf;
  dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
  dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
  dma_init_struct.number = RX_BUF_SIZE;
  dma_init_struct.periph_addr = USART_DATA(uart);
  dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
  dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
  dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
  dma_init(dma_rx, dma_rx_channel, &dma_init_struct);

  dma_circulation_enable(dma_rx, dma_rx_channel);   
  dma_channel_enable(dma_rx, dma_rx_channel);

  usart_dma_receive_config(uart, USART_DENR_ENABLE);

  reset_rx_buffer();
}

// Read new char in RX buffer:
int BufferedSerial::read(void) {
  // Get DMA head:
  uint16_t dma_head = (RX_BUF_SIZE - DMA_CHCNT(dma_rx, dma_rx_channel)) & rx_buf_mask;

  // Quit if no new character:
  if (dma_head == rx_tail) return -1;

  // Get next char in buffer:
  int c = (int) rx_buf[rx_tail++];

  // Wrap around if rx_tail > RX_BUF_SIZE:
  rx_tail &= rx_buf_mask;

  return c;
}

// Prepare data and send it:
void BufferedSerial::write(const uint8_t *data, const int length) {
  // If data can fit at the end of the buffer:
  if (tx_head + length < TX_BUF_SIZE) {
    memcpy(&(tx_buf[tx_head]), data, length);
    tx_head += length; // ) & tx_buf_mask
    if (tx_head > tx_end) tx_end = tx_head; // Avoids "wrong checksum for topic id and msg" in rosserial logs
  }
  // Else data is copied at the beginning of TX buffer:
  else {
    memcpy(tx_buf, data, length);
    if (tx_head > tx_tail) tx_end = tx_head; // Avoids tx_end > tx_tail
    tx_head = length;
  }

  // Send data:
  flush_tx_buffer();
}

// Send data:
void BufferedSerial::flush_tx_buffer(void) {
  static bool mutex = false;

  // Reset indexes if they are at the same position:
  if (tx_head != 0 && tx_head == tx_tail) {   // Can be removed (just for better memory management)
     tx_head = 0;
     tx_tail = 0;
   }

  // Quit if UART not ready to transmit data or no data to send:
  if (!usart_flag_get(uart, USART_FLAG_TC) || tx_head == tx_tail || mutex) return;
  mutex = true;

  // Reset flush index if already sent complete TX buffer:
  if (tx_tail == tx_end) tx_tail = 0;

  // Send data behind head:
  if (tx_tail < tx_head) {
    uint16_t length = tx_head - tx_tail;
    uart_transmit_dma(&huart, &(tx_buf[tx_tail]), length);
    tx_tail = tx_head;
  }
  // Else end the buffer before resetting tail index:
  else {
    uint16_t length = tx_end - tx_tail;
    uart_transmit_dma(&huart, &(tx_buf[tx_tail]), length);

    // Reset indexes:
    tx_end = TX_BUF_SIZE;
    tx_tail = 0;
  }

  mutex = false;
}

void uart_transmit_dma(uint16_t length){
  usart_flag_clear(uart, USART_FLAG_TC);
  dma_transfer_number_config(dma_tx, dma_tx_channel,length);
  /* USART DMA enable for transmission and reception */
  usart_dma_transmit_config(uart, USART_TRANSMIT_DMA_ENABLE);
}

// Reset DMA to the beginning of the RX buffer:
inline void BufferedSerial::reset_rx_buffer(void) {
  dma_transfer_number_config(dma_rx, dma_rx_channel,RX_BUF_SIZE);
}

// // DMA callbacks:
// void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
//   // Comparing pointers: (remove equality if only one UART is used)
//   if (huart->Instance == buff_serial.get_handle()->Instance) {
//     buff_serial.flush_tx_buffer();
//   }
// }

