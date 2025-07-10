/*
 * GD32Hardware.h
 *
 *  Created on: 10/07/2025
 *      Author: nekraus
 */

#ifndef ROS_GD32_HARDWARE_H_
#define ROS_GD32_HARDWARE_H_

#include "BufferedSerial.hpp"

// Create Serial Buffer with UART2:
extern BufferedSerial buff_serial;

class GD32Hardware {
 public:
  GD32Hardware() : serial(&buff_serial) {}

  // Any initialization code necessary to use the serial port:
  void init() { serial->init(); }

  // Read a byte from the serial port. -1 = failure:
  int read() { return serial->read(); }

  // Write data to the connection to ROS:
  void write(uint8_t* data, int length) { serial->write(data, length); }

  // Returns milliseconds since start of program:
  unsigned long time() { return tx_time_get(); };

 protected:
  BufferedSerial* serial;
};

#endif
