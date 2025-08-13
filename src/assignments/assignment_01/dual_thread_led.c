#include "contiki.h"
#include "dev/leds.h"
#include <stdio.h>

PROCESS(thread1_process, "Thread 1 - RED LED");
PROCESS(thread2_process, "Thread 2 - GREEN LED");
AUTOSTART_PROCESSES(&thread1_process, &thread2_process);

/* =========================== Thread 1 =========================== */
/* Toggle RED every 3s, i += 2 */
PROCESS_THREAD(thread1_process, ev, data)
{
  static struct etimer timer1;
  static int i = 0;

  PROCESS_BEGIN();

  etimer_set(&timer1, CLOCK_SECOND * 3);

  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer1));

    leds_toggle(LEDS_RED);
    printf("[Thread 1][%lu s] i = %d | RED LED %s\n", 
           clock_seconds(), 
           i, 
           (leds_get() & LEDS_RED) ? "ON" : "OFF");

    i += 2;
    etimer_reset(&timer1);
  }

PROCESS_END();
}

/* =========================== Thread 2 =========================== */
/* Toggle GREEN every 5s, j += 5 */
PROCESS_THREAD(thread2_process, ev, data)
{
  static struct etimer timer2;
  static int j = 0;

  PROCESS_BEGIN();

  etimer_set(&timer2, CLOCK_SECOND * 5);

  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer2));

    leds_toggle(LEDS_GREEN);
    printf("[Thread 2][%lu s] j = %d | GREEN LED %s\n", 
           clock_seconds(), 
           j,
           (leds_get() & LEDS_GREEN) ? "ON" : "OFF");

    j += 5;
    etimer_reset(&timer2);
  }

PROCESS_END();
}
