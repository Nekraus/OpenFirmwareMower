/*
 * ros.h
 *
 *  Created on: Nov 2, 2019
 *      Author: xav-jann1
 */

#ifndef _ROS_H_
#define _ROS_H_

#include "GD32Hardware.h"
#include "ros/node_handle.h"

namespace ros {
  typedef NodeHandle_<GD32Hardware> NodeHandle;  // default 25, 25, 512, 512
}

#endif
