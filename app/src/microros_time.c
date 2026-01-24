#include <time.h>

#include "tx_api.h"

#define TX_TIMER_NANOSECOND_PER_TICK ((ULONG)(1000000000UL / TX_TIMER_TICKS_PER_SECOND))

int clock_gettime(clockid_t t, struct timespec *tspec)
{
  ULONG tx_time;

  tx_time = tx_time_get();

  tspec->tv_sec = tx_time / TX_TIMER_TICKS_PER_SECOND;
  tx_time -= tspec->tv_sec * TX_TIMER_TICKS_PER_SECOND;
  tspec->tv_nsec = tx_time * TX_TIMER_NANOSECOND_PER_TICK;

  return 0;
}