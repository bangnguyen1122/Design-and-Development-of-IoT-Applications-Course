#include "contiki.h"
#include "dev/leds.h"
#include <stdio.h>

PROCESS(led_toggle_process, "LED Toggle with Single Thread");
AUTOSTART_PROCESSES(&led_toggle_process);

PROCESS_THREAD(led_toggle_process, ev, data)
{
  static struct etimer timer_red, timer_green;

  PROCESS_BEGIN();

  etimer_set(&timer_red, CLOCK_SECOND * 3);
  etimer_set(&timer_green, CLOCK_SECOND * 5);

  while(1) {
    PROCESS_WAIT_EVENT();

    if(etimer_expired(&timer_red)) {

      leds_toggle(LEDS_RED);
      printf("[%lu s] RED LED %s\n", 
      clock_seconds(),
      (leds_get() & LEDS_RED) ? "ON" : "OFF");

      etimer_reset(&timer_red); 
    }

    if(etimer_expired(&timer_green)) {

      leds_toggle(LEDS_GREEN);
      printf("[%lu s] GREEN LED %s\n", 
      clock_seconds(),
      (leds_get() & LEDS_GREEN) ? "ON" : "OFF");

      etimer_reset(&timer_green);
    }
  }

  PROCESS_END();
}

