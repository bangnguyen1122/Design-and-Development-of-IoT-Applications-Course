#include "contiki.h"
#include "net/rime/rime.h"
#include "random.h"
#include "dev/leds.h"
#include "sys/ctimer.h"
#include <stdio.h>
#include <string.h>

/* ===================== Configuration ===================== */
#define MAX_NEIGHBORS        3                      
#define BEACON_INTERVAL      (CLOCK_SECOND * 10)    
#define DATA_INTERVAL        (CLOCK_SECOND * 10)    
#define OF_DECAY_INTERVAL    (CLOCK_SECOND * 20)    
#define OF_DECAY_STEP        1                      
#define MAX_BEACON_HOPS      6                     

/* Stabilizers */
#define RSSI_HYST_DB         3                      
#define HOLD_WINDOW          (CLOCK_SECOND * 20)   

/* Root link-layer address (u8[0].u8[1]) */
#define ROOT_ID_0 1
#define ROOT_ID_1 0

/* ================== Helpers for seq (uint16_t) ================= */
static inline uint16_t seq_dist(uint16_t a, uint16_t b) {
  return (uint16_t)(b - a);
}
static inline int seq_newer(uint16_t new_s, uint16_t ref_s) {
  return (new_s != ref_s) && (seq_dist(ref_s, new_s) < 32768);
}

/* ================== Neighbor Data Structure ================= */
typedef struct {
  linkaddr_t   addr;         
  int16_t      rssi;          
  uint16_t     rx_unique;      
  uint16_t     rx_counter;     
  uint16_t     dup_count;    
  uint16_t     first_seq;     
  uint16_t     last_seq;    
  uint16_t     tx_est;         
  uint16_t     prr1000;        
  clock_time_t last_update_time;
  clock_time_t lock_until;    
} neighbor_t;

static neighbor_t neighbor_table[MAX_NEIGHBORS];
static uint8_t    neighbor_count = 0;

/* ===================== Packet Structures (PACKED) ===================== */
typedef struct {
  uint16_t seq;        
  uint8_t  origin[2]; 
  uint8_t  hop;        
} __attribute__((packed)) beacon_packet_t;

typedef struct {
  uint16_t seq;
  uint8_t  sender[2];
  uint8_t  ttl;
  int16_t  value;
} __attribute__((packed)) data_packet_t;

/* ===================== Globals & Connections ===================== */
static struct broadcast_conn beacon_bc;
static struct unicast_conn   data_uc;

static uint16_t seq_id = 0;                 
static uint16_t last_flooded_root_seq = 0;  

static struct ctimer rb_ctimer;
static beacon_packet_t rb_pkt_pending;
static uint8_t rb_pending = 0;

static int is_root_node(void) {
  return linkaddr_node_addr.u8[0] == ROOT_ID_0 && linkaddr_node_addr.u8[1] == ROOT_ID_1;
}

/* ================== Neighbor Table Operations ================= */
static int find_neighbor(const linkaddr_t *addr) {
  for(uint8_t i = 0; i < neighbor_count; i++) {
    if(linkaddr_cmp(&neighbor_table[i].addr, addr)) return i;
  }
  return -1;
}

static void remove_neighbor(uint8_t idx) {
  if(idx < neighbor_count - 1) {
    memcpy(&neighbor_table[idx], &neighbor_table[neighbor_count - 1], sizeof(neighbor_t));
  }
  neighbor_count--;
}

static void sort_neighbors_by_rssi(void) {
  for(uint8_t i = 0; i + 1 < neighbor_count; i++) {
    for(uint8_t j = i + 1; j < neighbor_count; j++) {
      if(neighbor_table[j].rssi > neighbor_table[i].rssi) {
        neighbor_t tmp = neighbor_table[i];
        neighbor_table[i] = neighbor_table[j];
        neighbor_table[j] = tmp;
      }
    }
  }
}

static void print_neighbor_table(void) {
  printf("Node %u.%u - Neighbors (max %u):\n",
         linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1], MAX_NEIGHBORS);
  printf("| Addr |  RSSI |  PRR(%%) | RX_u | TX_est | RX_ctr |\n");
  for(uint8_t i = 0; i < neighbor_count; i++) {
    const neighbor_t *n = &neighbor_table[i];
    printf("| %u.%u | %5d | %3u.%03u | %4u | %6u | %6u |\n",
           n->addr.u8[0], n->addr.u8[1],
           n->rssi,
           n->prr1000/1000, n->prr1000%1000,
           n->rx_unique,
           n->tx_est,
           n->rx_counter);
  }
}

/* ---- Update with hysteresis + hold-down + EWMA RSSI ---- */
static void update_neighbor(const linkaddr_t *addr, int16_t rssi_new, uint16_t root_seq) {
  clock_time_t now = clock_time();
  int idx = find_neighbor(addr);

  if(idx == -1) {
    /* Not in table yet */
    if(neighbor_count < MAX_NEIGHBORS) {
      idx = neighbor_count++;
    } else {
      /* Select victim = weakest RSSI */
      int victim = 0;
      for(int i = 1; i < MAX_NEIGHBORS; i++) {
        if(neighbor_table[i].rssi < neighbor_table[victim].rssi) victim = i;
      }
      int16_t victim_rssi = neighbor_table[victim].rssi;
      int allow_replace = 0;

      if(rssi_new >= victim_rssi + RSSI_HYST_DB) {
        allow_replace = 1; 
      } else if(now >= neighbor_table[victim].lock_until) {
        /* Hold-down expired but still not >= hysteresis: keep stability, do NOT replace */
        allow_replace = 0;
      } else {
        allow_replace = 0; 
      }

      if(!allow_replace) {
        return; 
      }
      idx = victim;
    }

    /* Initialize entry */
    linkaddr_copy(&neighbor_table[idx].addr, addr);
    neighbor_table[idx].rssi         = rssi_new;
    neighbor_table[idx].first_seq    = root_seq;
    neighbor_table[idx].last_seq     = root_seq;
    neighbor_table[idx].rx_unique    = 1;
    neighbor_table[idx].dup_count    = 0;
    neighbor_table[idx].tx_est       = 1;
    neighbor_table[idx].prr1000      = 1000;
    neighbor_table[idx].rx_counter   = 1;   
    neighbor_table[idx].last_update_time = now;
    neighbor_table[idx].lock_until   = now + HOLD_WINDOW; 
    return;
  }

  /* Already in table: update in place */
  neighbor_t *n = &neighbor_table[idx];
  /* EWMA smoothing: 3/4 old + 1/4 new */
  n->rssi = (int16_t)((3 * n->rssi + rssi_new) / 4);
  n->last_update_time = now;

  if(root_seq == n->last_seq) {
    n->dup_count++; /* duplicate */
  } else if(seq_newer(root_seq, n->last_seq)) {
    n->rx_unique++;
    n->last_seq = root_seq;
  } else {
    /* older/out-of-order: ignore */
  }

  if(n->rx_counter < UINT16_MAX) n->rx_counter++; 

  n->tx_est  = (uint16_t)(seq_dist(n->first_seq, n->last_seq) + 1);
  n->prr1000 = (n->tx_est > 0) ? (uint16_t)((1000UL * n->rx_unique) / n->tx_est) : 0;
}

/* Pick Best Next Neighbor (BNN): top-1 by RSSI */
static const linkaddr_t* get_bnn(void) {
  if(neighbor_count == 0) return NULL;
  return &neighbor_table[0].addr;
}

/* online_frequency: decay rx_counter every OF_DECAY_INTERVAL */
static void of_decay_tick(void) {
  uint8_t i = 0;
  while(i < neighbor_count) {
    if(neighbor_table[i].rx_counter > 0) {
      if(neighbor_table[i].rx_counter > OF_DECAY_STEP) neighbor_table[i].rx_counter -= OF_DECAY_STEP;
      else neighbor_table[i].rx_counter = 0;
    }
    if(neighbor_table[i].rx_counter == 0) {
      remove_neighbor(i);
    } else {
      i++;
    }
  }
  sort_neighbors_by_rssi();
}

/* ==================== Broadcast (Beacon) ==================== */
static void do_rebroadcast(void *ptr) {
  if(!rb_pending) return;
  packetbuf_copyfrom(&rb_pkt_pending, sizeof(rb_pkt_pending));
  broadcast_send(&beacon_bc);
  printf("Rebcast beacon: seq=%u hop=%u\n", rb_pkt_pending.seq, rb_pkt_pending.hop);
  rb_pending = 0;
}

static void beacon_recv(struct broadcast_conn *c, const linkaddr_t *from) {
  beacon_packet_t pkt;
  memcpy(&pkt, packetbuf_dataptr(), sizeof(pkt));

  int16_t rssi = (int16_t)packetbuf_attr(PACKETBUF_ATTR_RSSI);

  /* Only process beacons of our root */
  if(pkt.origin[0] == ROOT_ID_0 && pkt.origin[1] == ROOT_ID_1) {
    /* Update neighbor stats for the 1-hop neighbor who sent this copy */
    update_neighbor(from, rssi, pkt.seq);
    sort_neighbors_by_rssi();
    print_neighbor_table();

    /* Controlled flood: re-broadcast root beacon outward (non-root nodes only) with jitter */
    if(!is_root_node()) {
      if(seq_newer(pkt.seq, last_flooded_root_seq) && pkt.hop < MAX_BEACON_HOPS) {
        last_flooded_root_seq = pkt.seq;

        rb_pkt_pending = pkt;
        rb_pkt_pending.hop = pkt.hop + 1;

        /* jitter ~20–70 ms to reduce collisions */
        clock_time_t jitter = (CLOCK_SECOND / 50) + (random_rand() % (CLOCK_SECOND / 20));
        rb_pending = 1;
        ctimer_set(&rb_ctimer, jitter, do_rebroadcast, NULL);
      }
    }
  }
}
static const struct broadcast_callbacks beacon_cb = { beacon_recv };

/* ==================== Unicast (Data Forwarding) ==================== */
static void data_recv(struct unicast_conn *c, const linkaddr_t *from) {
  data_packet_t pkt;
  memcpy(&pkt, packetbuf_dataptr(), sizeof(pkt));

  if(is_root_node()) {
    printf("ROOT RX data: Seq=%u from %u.%u TTL=%u Value=%d\n",
           pkt.seq, pkt.sender[0], pkt.sender[1], pkt.ttl, pkt.value);
  } else {
    if(pkt.ttl > 0) {
      pkt.ttl--;
      const linkaddr_t *next_hop = get_bnn();
      if(next_hop && !linkaddr_cmp(next_hop, from)) { 
        packetbuf_copyfrom(&pkt, sizeof(pkt));
        unicast_send(&data_uc, next_hop);
        printf("FWD data: from %u.%u -> %u.%u (orig %u.%u) TTL=%u\n",
               from->u8[0], from->u8[1], next_hop->u8[0], next_hop->u8[1],
               pkt.sender[0], pkt.sender[1], pkt.ttl);
      }
    }
  }
}
static const struct unicast_callbacks data_cb = { data_recv };

/* ====================== Main Process ====================== */
PROCESS(tree_routing_bnn_rssi_process, "Tree Routing with BNN & RSSI (stable)");
AUTOSTART_PROCESSES(&tree_routing_bnn_rssi_process);

PROCESS_THREAD(tree_routing_bnn_rssi_process, ev, data)
{
  static struct etimer beacon_timer, data_timer, of_timer;

  PROCESS_EXITHANDLER({
    broadcast_close(&beacon_bc);
    unicast_close(&data_uc);
  });

  PROCESS_BEGIN();

  broadcast_open(&beacon_bc, 129, &beacon_cb);
  unicast_open(&data_uc, 146, &data_cb);

  etimer_set(&beacon_timer, BEACON_INTERVAL);
  etimer_set(&data_timer,   DATA_INTERVAL);
  etimer_set(&of_timer,     OF_DECAY_INTERVAL);

  while(1) {
    PROCESS_WAIT_EVENT();

    /* Root emits original beacons every 10s */
    if(etimer_expired(&beacon_timer)) {
      if(is_root_node()) {
        beacon_packet_t b;
        b.seq = ++seq_id;              
        b.origin[0] = ROOT_ID_0;
        b.origin[1] = ROOT_ID_1;
        b.hop = 0;

        packetbuf_copyfrom(&b, sizeof(b));
        broadcast_send(&beacon_bc);
        printf("ROOT beacon: seq=%u\n", b.seq);

        /* helps flood filter logic across nodes */
        last_flooded_root_seq = b.seq;
      }
      etimer_reset(&beacon_timer);
    }

    /* Each non-root node sends data to its BNN every 10s */
    if(etimer_expired(&data_timer)) {
      if(!is_root_node()) {
        const linkaddr_t *next = get_bnn();
        if(next) {
          data_packet_t d;
          d.seq = ++seq_id;
          d.sender[0] = linkaddr_node_addr.u8[0];
          d.sender[1] = linkaddr_node_addr.u8[1];
          d.ttl = 10;
          d.value = (int16_t)(random_rand() % 100);

          packetbuf_copyfrom(&d, sizeof(d));
          unicast_send(&data_uc, next);
          printf("TX data -> %u.%u: Seq=%u Val=%d\n",
                 next->u8[0], next->u8[1], d.seq, d.value);
        } else {
          printf("No BNN available; data not sent.\n");
        }
      }
      etimer_reset(&data_timer);
    }

    /* online_frequency decay tick */
    if(etimer_expired(&of_timer)) {
      of_decay_tick();
      etimer_reset(&of_timer);
    }
  }

  PROCESS_END();
}
