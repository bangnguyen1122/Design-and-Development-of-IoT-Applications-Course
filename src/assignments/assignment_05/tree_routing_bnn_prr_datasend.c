#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include "net/rime/rime.h"
#include "dev/leds.h"
#include "node-id.h"
#include "dev/sht11/sht11-sensor.h"
#include "lib/random.h"

/*==================== Message Formats ====================*/
/* Beacon from root and forwarders */
typedef struct {
  unsigned short adv_parent; 
  uint16_t       adv_hops;    
  uint16_t       adv_seq;      
} beacon_msg_t;

/* Application data */
typedef struct {
  unsigned short src;         
  uint16_t       hops;         
  uint16_t       temp_raw;    
  uint16_t       data_id;     
} data_msg_t;

/* ACK for unicast data */
typedef struct {
  unsigned short ack_from;    
  uint16_t       data_id;      
  uint8_t        ok;          
} ack_msg_t;

/*==================== Neighbor Record ====================*/
typedef struct {
  unsigned short id;         
  uint16_t       tx;           
  uint16_t       rx_ack;       
  int            rssi;         
  uint16_t       hops_via;     
  float          prr;          
  uint8_t        used;        
  clock_time_t   seen_at;      
} nbr_t;

/*======================== Constants =======================*/
#define SINK_ID                 1
#define CH_BC                   128
#define CH_DATA                 140
#define CH_ACK                  142
#define T_STARTUP_WAIT          5
#define T_BC                    45     
#define T_PRINT                 28     
#define T_DATA                  60
#define T_RESELECT              9       
#define T_AGING                 60

#define HOPS_MAX                20
#define NBR_CAP                 10
#define PRR_MIN_SAMPLES         3
#define NBR_TTL                 (180 * CLOCK_SECOND)

#define PICK_HOP                1
#define PICK_RSSI               2
#define PICK_PRR                3
#ifndef PICK_POLICY
#define PICK_POLICY             PICK_PRR
#endif

/*===================== Module State ======================*/
static struct broadcast_conn bc;
static struct unicast_conn   uc_data;
static struct unicast_conn   uc_ack;

static unsigned short next_hop = 0;      
static uint16_t       data_seq = 0;      
static uint16_t       disc_seq_tx = 0;  
static uint16_t       disc_seq_rx = 0;  

static struct etimer  et0, et1, et2, et3;
static struct ctimer  led_off;

static nbr_t          nbrs[NBR_CAP];
static short          hop_hist[HOPS_MAX];
static radio_value_t  rtmp;

/*======================== Prototypes =====================*/
static void    led_off_cb(void);
static void    nbr_init(void);
static int     nbr_find(unsigned short id);
static void    nbr_touch(nbr_t *n);
static void    nbr_upsert(unsigned short id, int rssi, uint16_t hops);
static void    nbr_expire(void);
static void    prr_bump(unsigned short id, uint8_t got_ack);
static void    parent_set(unsigned short id);
static void    data_send(data_msg_t *m);
static void    temp_print(uint16_t raw);

static void    cb_bc(struct broadcast_conn *c, linkaddr_t *from);
static void    cb_uc_data(struct unicast_conn *c, const linkaddr_t *from);
static void    cb_uc_ack(struct unicast_conn *c, const linkaddr_t *from);

static void    parent_reselect(void);

/*======================== Processes ======================*/
PROCESS(proc_route,  "Routing / Beacon");
PROCESS(proc_data,   "Data TX/RX");
PROCESS(proc_pick,   "Parent Selection");
PROCESS(proc_stats,  "Stats / Debug");

AUTOSTART_PROCESSES(&proc_route, &proc_data, &proc_pick, &proc_stats);

/*======================== Utilities ======================*/
static void led_off_cb(void){ leds_off(LEDS_BLUE); }

static void nbr_init(void){
  for(int i=0;i<NBR_CAP;i++){
    nbrs[i].used = 0; nbrs[i].id = 0; nbrs[i].tx = nbrs[i].rx_ack = 0;
    nbrs[i].rssi = -127; nbrs[i].hops_via = UINT16_MAX; nbrs[i].prr = 0.f; nbrs[i].seen_at = 0;
  }
}

static int nbr_find(unsigned short id){
  for(int i=0;i<NBR_CAP;i++) if(nbrs[i].used && nbrs[i].id==id) return i;
  return -1;
}

static void nbr_touch(nbr_t *n){ n->used=1; n->seen_at = clock_time(); }

static void nbr_upsert(unsigned short id, int rssi, uint16_t hops){
  int k = nbr_find(id);
  if(k>=0){
    nbrs[k].rssi = rssi; nbrs[k].hops_via = hops; nbr_touch(&nbrs[k]); return;
  }
  for(int i=0;i<NBR_CAP;i++){
    if(!nbrs[i].used){
      nbrs[i].id=id; nbrs[i].rssi=rssi; nbrs[i].hops_via=hops;
      nbrs[i].tx=nbrs[i].rx_ack=0; nbrs[i].prr=0.f; nbr_touch(&nbrs[i]); return;
    }
  }

  int oldest=0; for(int i=1;i<NBR_CAP;i++) if(nbrs[i].seen_at<nbrs[oldest].seen_at) oldest=i;
  nbrs[oldest].id=id; nbrs[oldest].rssi=rssi; nbrs[oldest].hops_via=hops;
  nbrs[oldest].tx=nbrs[oldest].rx_ack=0; nbrs[oldest].prr=0.f; nbr_touch(&nbrs[oldest]);
}

static void nbr_expire(void){
  clock_time_t now = clock_time();
  for(int i=0;i<NBR_CAP;i++){
    if(nbrs[i].used && (now - nbrs[i].seen_at > NBR_TTL)){
      if(nbrs[i].id == next_hop){
        printf("[aging] parent %u expired; reset\n", next_hop);
        next_hop = 0;
      }
      nbrs[i].used = 0;
    }
  }
}

static void prr_bump(unsigned short id, uint8_t got_ack){
  int k = nbr_find(id); if(k<0) return;
  if(got_ack) nbrs[k].rx_ack++;
  nbrs[k].tx++;
  if(nbrs[k].tx) nbrs[k].prr = (float)nbrs[k].rx_ack/(float)nbrs[k].tx;
}

static void parent_set(unsigned short id){
  if(next_hop != id){
    next_hop = id;
    int k = nbr_find(id);
    int prr_i = (k>=0) ? (int)(nbrs[k].prr*100.f) : -1;
    printf("[route] parent=%u (hop=%u rssi=%d prr=%d%%)\n",
           next_hop, (k>=0?nbrs[k].hops_via:0), (k>=0?nbrs[k].rssi:0), prr_i);
  }
}

static void temp_print(uint16_t raw){
  printf("%d.%d", (raw / 10 - 396) / 10, (raw / 10 - 396) % 10);
}

static void data_send(data_msg_t *m){
  packetbuf_clear();
  packetbuf_copyfrom(m, sizeof(*m));
  linkaddr_t nh; nh.u8[0]=next_hop; nh.u8[1]=0;
  unicast_send(&uc_data, &nh);
  prr_bump(next_hop, 0); 
}

/*======================== Callbacks ======================*/
static void cb_bc(struct broadcast_conn *c, linkaddr_t *from){
  if(node_id == SINK_ID) return;

  beacon_msg_t b; packetbuf_copyto(&b);
  /* grab RSSI attr */
  NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, &rtmp);
  rtmp = packetbuf_attr(PACKETBUF_ATTR_RSSI);
  int rssi = (int8_t)rtmp;

  printf("[beacon] from=%u seq=%u hop=%u rssi=%d\n",
         from->u8[0], b.adv_seq, b.adv_hops, rssi);

  /* record advertiser as candidate */
  nbr_upsert(b.adv_parent, rssi, b.adv_hops);

  static uint16_t prev = 0;
  uint8_t fwd = 0;
  if(prev==0){                 
    parent_set(b.adv_parent); fwd=1;
  } else if(b.adv_seq > prev){ fwd=1; }
  prev = b.adv_seq;

  if(fwd){
    beacon_msg_t out = { node_id, (uint16_t)(b.adv_hops+1), b.adv_seq };
    packetbuf_copyfrom(&out, sizeof(out));
    broadcast_send(&bc);
    printf("[beacon] fwd seq=%u newhop=%u\n", out.adv_seq, out.adv_hops);
  }
}

static void cb_uc_data(struct unicast_conn *c, const linkaddr_t *from){
  data_msg_t d; packetbuf_copyto(&d);

  /* reply ACK */
  ack_msg_t a = { node_id, d.data_id, 1 };
  packetbuf_clear(); packetbuf_copyfrom(&a, sizeof(a));
  unicast_send(&uc_ack, from);

  /* mark child */
  int k = nbr_find(from->u8[0]);
  if(k>=0) nbr_touch(&nbrs[k]);

  if(node_id == SINK_ID){
    if(d.hops < HOPS_MAX) hop_hist[d.hops]++;
    printf("[sink] recv src=%u hops=%u temp=", d.src, d.hops);
    temp_print(d.temp_raw);
    printf("\n");
  }else{
    /* forward upwards */
    d.hops++;
    data_send(&d);
    printf("[relay] me=%u fwd src=%u -> parent=%u\n", node_id, d.src, next_hop);
  }
}

static void cb_uc_ack(struct unicast_conn *c, const linkaddr_t *from){
  ack_msg_t a; packetbuf_copyto(&a);
  prr_bump(from->u8[0], 1);
  int k = nbr_find(from->u8[0]); if(k>=0) nbr_touch(&nbrs[k]);
  printf("[ack] from=%u data=%u\n", from->u8[0], a.data_id);
}

/*======================= Selection =======================*/
static inline float score_hop(const nbr_t *n){
  if(n->hops_via==UINT16_MAX) return -1.f;
  return 1.f/(1.f + n->hops_via);
}
static inline float score_rssi(const nbr_t *n){ return (float)n->rssi; }
static inline float score_prr (const nbr_t *n){
  return (n->tx < PRR_MIN_SAMPLES) ? -1.f : n->prr;
}

static void parent_reselect(void){
  nbr_t *best=NULL; float s_best=-1.f;

  for(int i=0;i<NBR_CAP;i++){
    if(!nbrs[i].used) continue;

#if PICK_POLICY == PICK_PRR
    float s = score_prr(&nbrs[i]);
#elif PICK_POLICY == PICK_RSSI
    float s = score_rssi(&nbrs[i]);
#else
    float s = score_hop(&nbrs[i]);
#endif

    if(s > s_best){ best=&nbrs[i]; s_best=s; }
    else if(s == s_best && best){
      if(nbrs[i].hops_via < best->hops_via) best=&nbrs[i];
      else if(nbrs[i].hops_via == best->hops_via && nbrs[i].rssi > best->rssi) best=&nbrs[i];
      else if(nbrs[i].hops_via == best->hops_via && nbrs[i].rssi == best->rssi && nbrs[i].id < best->id) best=&nbrs[i];
    }
  }

#if PICK_POLICY == PICK_PRR
  /* fallback if no neighbor has enough PRR samples */
  if(!best){
    for(int i=0;i<NBR_CAP;i++){
      if(!nbrs[i].used) continue;
      float s = score_hop(&nbrs[i]);
      if(s > s_best){ best=&nbrs[i]; s_best=s; }
      else if(s == s_best && best){
        if(nbrs[i].hops_via < best->hops_via) best=&nbrs[i];
        else if(nbrs[i].hops_via == best->hops_via && nbrs[i].rssi > best->rssi) best=&nbrs[i];
      }
    }
  }
#endif

  if(best) parent_set(best->id);
}

/*======================== Processes ======================*/
static const struct broadcast_callbacks bc_cb = { cb_bc };
static const struct unicast_callbacks  uc_data_cb = { cb_uc_data };
static const struct unicast_callbacks  uc_ack_cb  = { cb_uc_ack };

PROCESS_THREAD(proc_route, ev, data){
  PROCESS_EXITHANDLER(broadcast_close(&bc);)
  PROCESS_BEGIN();

  broadcast_open(&bc, CH_BC, &bc_cb);
  memset(hop_hist, 0, sizeof(hop_hist));
  nbr_init();
  next_hop = 0; disc_seq_rx = 0;

  if(node_id == SINK_ID){
    /* sink emits beacons forever */
    etimer_set(&et0, T_STARTUP_WAIT * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et0));

    while(1){
      beacon_msg_t b = { SINK_ID, 1, ++disc_seq_tx };
      packetbuf_copyfrom(&b, sizeof(b));
      broadcast_send(&bc);

      leds_on(LEDS_BLUE);
      ctimer_set(&led_off, CLOCK_SECOND/8, led_off_cb, NULL);

      etimer_set(&et0, T_BC * CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et0));
    }
  }else{
    while(1) PROCESS_WAIT_EVENT();
  }

  PROCESS_END();
}

PROCESS_THREAD(proc_data, ev, data){
  PROCESS_EXITHANDLER(unicast_close(&uc_data); unicast_close(&uc_ack);)
  PROCESS_BEGIN();

  unicast_open(&uc_data, CH_DATA, &uc_data_cb);
  unicast_open(&uc_ack,  CH_ACK,  &uc_ack_cb);
  SENSORS_ACTIVATE(sht11_sensor);

  /* small desync based on id */
  etimer_set(&et1, (node_id % T_DATA) * CLOCK_SECOND);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et1));

  while(1){
    etimer_set(&et1, T_DATA * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et1));

    if(node_id != SINK_ID && next_hop){
      data_msg_t d = { node_id, 1, sht11_sensor.value(SHT11_SENSOR_TEMP), ++data_seq };
      data_send(&d);
      printf("[tx] node=%u -> %u id=%u\n", node_id, next_hop, data_seq);
    }else if(node_id == SINK_ID){
      hop_hist[0]++; 
    }
  }

  PROCESS_END();
}

PROCESS_THREAD(proc_pick, ev, data){
  PROCESS_BEGIN();
  while(1){
    etimer_set(&et2, T_RESELECT * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et2));
    nbr_expire();
    if(node_id != SINK_ID) parent_reselect();
  }
  PROCESS_END();
}

PROCESS_THREAD(proc_stats, ev, data){
  PROCESS_BEGIN();
  while(1){
    etimer_set(&et3, T_PRINT * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et3));
    if(node_id == SINK_ID){
      printf("[hops] "); for(int i=0;i<HOPS_MAX;i++) printf("%d ", hop_hist[i]); printf("\n");
    }else{
      printf("[tbl] node=%u parent=%u policy=%d\n", node_id, next_hop, PICK_POLICY);
      printf(" id  hop rssi tx ack prr%%\n");
      for(int i=0;i<NBR_CAP;i++){
        if(!nbrs[i].used || nbrs[i].hops_via==UINT16_MAX) continue;
        int prr_i = (int)( (nbrs[i].tx? (nbrs[i].prr*100.f) : 0) );
        printf(" %-3u %-3u %-4d %-3u %-3u %3d\n",
               nbrs[i].id, nbrs[i].hops_via, nbrs[i].rssi, nbrs[i].tx, nbrs[i].rx_ack, prr_i);
      }
    }
  }
  PROCESS_END();
}
