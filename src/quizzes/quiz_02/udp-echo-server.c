#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"

#include <string.h>
#include <stdlib.h>

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"
#include "dev/watchdog.h"
#include "dev/leds.h"
#include "net/rpl/rpl.h"
#include "dev/leds.h"

#include "sls.h"

/*---------------------------------------------------------------------------*/
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_UDP_BUF  ((struct uip_udp_hdr *)&uip_buf[uip_l2_l3_hdr_len])

#define MAX_PAYLOAD_LEN 120
/*---------------------------------------------------------------------------*/
static struct uip_udp_conn *server_conn;
static char buf[MAX_PAYLOAD_LEN];
static uint16_t len;

/* SLS define */
static struct led_struct_t led_db;
static struct net_struct_t net_db;

static char str_reply[50];
static char str_cmd[10];
static char str_arg[10];
static char str_rx[MAX_PAYLOAD_LEN];
  
static  radio_value_t aux;

static int8_t simulated_temp = 35;  

/* define prototype of function call */
static void get_radio_parameter(void);
static void init_default_parameters(void);
static void reset_parameters(void);
static  char *p;

/*---------------------------------------------------------------------------*/
PROCESS(udp_echo_server_process, "UDP echo server process");
AUTOSTART_PROCESSES(&udp_echo_server_process);
/*---------------------------------------------------------------------------*/
static void tcpip_handler(void) {

  memset(buf, 0, MAX_PAYLOAD_LEN);
  if(uip_newdata()) {
    leds_on(LEDS_RED);
    len = uip_datalen();
    memcpy(buf, uip_appdata, len);
    PRINTF("Received from [");
    PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
    PRINTF("]: %u\n", UIP_HTONS(UIP_UDP_BUF->srcport));
    PRINTF("%u bytes DATA: %s; ", len, buf);

    uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
    server_conn->rport = UIP_UDP_BUF->srcport;

    get_radio_parameter();
    reset_parameters();

    strcpy(str_rx, buf);
    if (SLS_CC2538DK_HW)
      sscanf(str_rx, "%s %s", str_cmd, str_arg);
    else {
      PRINTF("str_rx = %s", str_rx);
      p = strtok(str_rx, " ");
      if (p != NULL) {
        strcpy(str_cmd, p);
        p = strtok(NULL, " ,");
        if (p != NULL) { strcpy(str_arg, p); }
      }
    }

    PRINTF("CMD = %s ARG = %s\n", str_cmd, str_arg);

    if (strstr(str_cmd, SLS_LED_ON) != NULL) {
      PRINTF("Execute CMD = %s\n", SLS_LED_ON);
      leds_on(LEDS_RED);
      sprintf(str_reply, "Replied = %s", str_rx);
      led_db.status = LED_ON;
    }

    else if (strstr(str_cmd, SLS_LED_OFF) != NULL) {
      PRINTF("Execute CMD = %s\n", SLS_LED_OFF);
      leds_off(LEDS_RED);
      sprintf(str_reply, "Replied = %s", str_rx);
      led_db.status = LED_OFF;
    }

    else if (strstr(str_cmd, SLS_LED_ALL_ON) != NULL) {
      PRINTF("Execute CMD = %s\n", SLS_LED_ALL_ON);
      leds_on(LEDS_GREEN);
      leds_on(LEDS_RED);
      leds_on(LEDS_BLUE);
    }

    else if (strstr(str_cmd, SLS_LED_DIM) != NULL) {
      PRINTF("Execute CMD = %s to value %s", SLS_LED_DIM, str_arg);
      leds_toggle(LEDS_BLUE);
      sprintf(str_reply, "Replied = %s\n", str_rx);
      led_db.status = LED_DIM;
      led_db.dim = atoi(str_arg);
    }

    else if (strstr(str_cmd, SLS_GET_LED_STATUS) != NULL) {
      sprintf(str_reply, "Replied: id=%u;power=%u;temp=%d;dim=%u;status=0x%02X; \n", led_db.id,
            led_db.power,  led_db.temperature, led_db.dim, led_db.status);
    }
    else if (strstr(str_cmd, SLS_GET_NW_STATUS) != NULL) {
      sprintf(str_reply, "Replied: channel=%u;rssi=%ddBm;lqi=%u;tx_power=%ddBm;panid=0x%02X;  \n", 
            net_db.channel, net_db.rssi, net_db.lqi, net_db.tx_power, net_db.panid);
    }
    else if (strstr(str_cmd, SLS_GET_TEMP) != NULL) {
      /* Query temperature setting processing */
      simulated_temp++;
      if(simulated_temp > 40) simulated_temp = 35;
      sprintf(str_reply, "Replied: temperature=%dC\n", simulated_temp);
    }
    else {
      reset_parameters();
      sprintf(str_reply,"unknown cmd\n");
    }

    PRINTF("str_reply=%s \n", str_reply);

    /* Feedback data back to client */
    PRINTF("Echo back to [");
    PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
    PRINTF("]: %u %u bytes\n", UIP_HTONS(UIP_UDP_BUF->srcport), sizeof(str_reply));
    uip_udp_packet_send(server_conn, str_reply, sizeof(str_reply));
    uip_create_unspecified(&server_conn->ripaddr);
    server_conn->rport = 0;
  }
  return;
}

static void reset_parameters(void) {
  memset(&str_cmd[0], 0, sizeof(str_cmd));
  memset(&str_arg[0], 0, sizeof(str_arg));
  memset(&str_reply[0], 0, sizeof(str_reply));
}

/*---------------------------------------------------------------------------*/
static void get_radio_parameter() {
  NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, &aux);
  net_db.channel = (unsigned int) aux;

  aux = packetbuf_attr(PACKETBUF_ATTR_RSSI);
  net_db.rssi = (int8_t)aux;

  aux = packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY);
  net_db.lqi = aux;

  NETSTACK_RADIO.get_value(RADIO_PARAM_TXPOWER, &aux);
  net_db.tx_power = aux;
}

/*---------------------------------------------------------------------------*/
static void init_default_parameters(void) {
  led_db.id    = 0x20;        
  led_db.power  = 120;
  led_db.dim    = 80;
  led_db.status  = LED_ON; 
  led_db.temperature = 35;
  net_db.panid   = SLS_PAN_ID;
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_echo_server_process, ev, data){

  PROCESS_BEGIN();

  PRINTF("Initialization....\n");
  init_default_parameters();

  PRINTF("Starting UDP echo server\n");

  server_conn = udp_new(NULL, UIP_HTONS(0), NULL);
  udp_bind(server_conn, UIP_HTONS(3000));

  PRINTF("Listen port: 3000, TTL=%u\n", server_conn->ttl);

  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }
  }

  PROCESS_END();
}
