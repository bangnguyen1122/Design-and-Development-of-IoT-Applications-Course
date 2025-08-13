#include "contiki.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"
#include "dev/serial-line.h"
#include <stdio.h>
#include <string.h>

PROCESS(hw_interface_process, "HW Interface Example");
AUTOSTART_PROCESSES(&hw_interface_process);

PROCESS_THREAD(hw_interface_process, ev, data)
{
  static uint8_t green_led_state = 0; 

  PROCESS_BEGIN();

  /* Activate button sensor and serial interface */
  SENSORS_ACTIVATE(button_sensor);
  serial_line_init();

  while(1) {
    PROCESS_WAIT_EVENT();

    /* Handle button press: turn ON GREEN LED */
    if(ev == sensors_event && data == &button_sensor) {
      green_led_state = !green_led_state;

      if(green_led_state) {
        leds_on(LEDS_GREEN);
        printf("Button pressed: GREEN LED ON\n");
      } else {
        leds_off(LEDS_GREEN);
        printf("Button pressed: GREEN LED OFF\n");
      }
    }

    /* Handle serial commands */
    if(ev == serial_line_event_message && data != NULL) {
      char *input = (char *)data;

      /* Remove newline or carriage return characters from the input */
      size_t len = strlen(input);
      if(len > 0 && (input[len-1] == '\n' || input[len-1] == '\r')) {
        input[len-1] = '\0';
      }

      /* Compare input command and execute LED actions */
      if(strcmp(input, "ON RED") == 0) {
        leds_on(LEDS_RED);
        printf("Serial command: RED LED ON\n");
      }
      else if(strcmp(input, "OFF RED") == 0) {
        leds_off(LEDS_RED);
        printf("Serial command: RED LED OFF\n");
      }
      else {
        printf("Unknown command: %s\n", input);
      }
    }
  }

  PROCESS_END();
}
