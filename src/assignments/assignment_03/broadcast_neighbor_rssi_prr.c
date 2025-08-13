#include "contiki.h"
#include "net/rime/rime.h"
#include "random.h"
#include "dev/leds.h"
#include <stdio.h>
#include <string.h>

/* ===================== Configuration ===================== */
#define MAX_NEIGHBORS     5                     
#define NEIGHBOR_TIMEOUT  (CLOCK_SECOND * 6)   

/* ================== Neighbor Data Structure ================= */
typedef struct {
  linkaddr_t   addr;          
  int          last_rssi;      
  uint16_t     rx_unique;     
  uint16_t     dup_count;      
  uint16_t     first_seq;     
  uint16_t     last_seq;       
  uint16_t     tx_est;         
  uint16_t     prr1000;       
  clock_time_t last_update_time; 
} neighbor_t;

/* Neighbor table (max MAX_NEIGHBORS entries) */
static neighbor_t neighbor_table[MAX_NEIGHBORS];
static uint8_t    neighbor_count = 0;

/* Packet counter for this node's own transmissions */
static uint16_t tx_counter = 0;

/* ================== Sequence Number Helpers ================= */
/* Distance between two sequence numbers (handles wrap-around) */
static inline uint16_t seq_dist(uint16_t a, uint16_t b) {
  return (uint16_t)(b - a); 
}

/* Check if new_s is newer than ref_s, considering wrap-around */
static inline int seq_newer(uint16_t new_s, uint16_t ref_s) {
  return (new_s != ref_s) && (seq_dist(ref_s, new_s) < 32768);
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

/* Sort neighbors in descending order by RSSI */
static void sort_neighbors_by_rssi(void) {
  for(uint8_t i = 0; i + 1 < neighbor_count; i++) {
    for(uint8_t j = i + 1; j < neighbor_count; j++) {
      if(neighbor_table[j].last_rssi > neighbor_table[i].last_rssi) {
        neighbor_t tmp = neighbor_table[i];
        neighbor_table[i] = neighbor_table[j];
        neighbor_table[j] = tmp;
      }
    }
  }
}

/* Remove neighbors that haven't sent data for NEIGHBOR_TIMEOUT */
static void cleanup_neighbors(clock_time_t now) {
  uint8_t i = 0;
  while(i < neighbor_count) {
    if(now - neighbor_table[i].last_update_time > NEIGHBOR_TIMEOUT) {
      remove_neighbor(i);
    } else {
      i++;
    }
  }
}

/* =============== Add or Update a Neighbor Entry =============== */
static void add_or_update_neighbor(const linkaddr_t *addr, int rssi, uint16_t seq, clock_time_t now) {
  int idx = find_neighbor(addr);

  /* New neighbor: add to table or replace lowest RSSI entry */
  if(idx == -1) {
    if(neighbor_count < MAX_NEIGHBORS) {
      idx = neighbor_count++;
    } else {
      int min_rssi_idx = 0;
      for(int i = 1; i < MAX_NEIGHBORS; i++) {
        if(neighbor_table[i].last_rssi < neighbor_table[min_rssi_idx].last_rssi) {
          min_rssi_idx = i;
        }
      }
      idx = min_rssi_idx;  
    }

    /* Initialize new entry */
    linkaddr_copy(&neighbor_table[idx].addr, addr);
    neighbor_table[idx].last_rssi = rssi;
    neighbor_table[idx].first_seq = seq;
    neighbor_table[idx].last_seq  = seq;
    neighbor_table[idx].rx_unique = 1;
    neighbor_table[idx].dup_count = 0;
    neighbor_table[idx].tx_est    = 1;
    neighbor_table[idx].prr1000   = 1000;
    neighbor_table[idx].last_update_time = now;
    return;
  }

  /* Existing neighbor: update stats */
  neighbor_t *n = &neighbor_table[idx];
  n->last_rssi = rssi;
  n->last_update_time = now;

  if(seq == n->last_seq) {
    /* Duplicate packet */
    n->dup_count++;
  } else if(seq_newer(seq, n->last_seq)) {
    /* Newer packet */
    n->rx_unique++;
    n->last_seq = seq;
  } else {
    /* Out-of-order or old packet: ignore */
  }

  /* Update estimated TX count and PRR */
  n->tx_est = (uint16_t)(seq_dist(n->first_seq, n->last_seq) + 1);
  if(n->tx_est > 0) {
    n->prr1000 = (uint16_t)((1000UL * n->rx_unique) / n->tx_est);
  } else {
    n->prr1000 = 0;
  }
}

/* ==================== Print Neighbor Table ==================== */
static void print_neighbor_table(void) {
  printf("Node %u.%u — Neighbor stats (max %u nodes):\n",
         linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1], MAX_NEIGHBORS);
  printf("| Node |  RSSI |  PRR(%%) | RX_u | TX_est | Dup |\n");
  for(uint8_t i = 0; i < neighbor_count; i++) {
    const neighbor_t *n = &neighbor_table[i];
    printf("| %2u.%u | %5d | %3u.%03u | %4u | %6u | %3u |\n",
           n->addr.u8[0], n->addr.u8[1],
           n->last_rssi,
           n->prr1000/1000, n->prr1000%1000,
           n->rx_unique,
           n->tx_est,
           n->dup_count);
  }
}

/* ===================== Packet Structure ===================== */
typedef struct {
  uint16_t seq;         
  uint8_t  sender_id[2];
} __attribute__((packed)) my_packet_t;

/* ===================== Contiki Process ===================== */
PROCESS(example_broadcast_process, "Broadcast Neighbor Table Example");
AUTOSTART_PROCESSES(&example_broadcast_process);

/* Callback when a broadcast packet is received */
static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from) {
  my_packet_t pkt;
  memcpy(&pkt, packetbuf_dataptr(), sizeof(my_packet_t));

  /* RSSI in Contiki is stored in PACKETBUF_ATTR_RSSI */
  int rssi = (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI);

  printf("RX from %u.%u: Seq=%u, SenderID=%u.%u, RSSI=%d\n",
         from->u8[0], from->u8[1], pkt.seq, pkt.sender_id[0], pkt.sender_id[1], rssi);

  add_or_update_neighbor(from, rssi, pkt.seq, clock_time());
  sort_neighbors_by_rssi();
  print_neighbor_table();
}

static const struct broadcast_callbacks broadcast_call = { broadcast_recv };
static struct broadcast_conn broadcast;

PROCESS_THREAD(example_broadcast_process, ev, data)
{
  static struct etimer et;
  PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
  PROCESS_BEGIN();

  broadcast_open(&broadcast, 129, &broadcast_call);

  while(1) {
    /* Random send interval: 2–4 seconds */
    etimer_set(&et, CLOCK_SECOND * 2 + random_rand() % (CLOCK_SECOND * 2));
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    /* Prepare and send broadcast packet */
    tx_counter++;

    my_packet_t pkt;
    pkt.seq = tx_counter;
    pkt.sender_id[0] = linkaddr_node_addr.u8[0];
    pkt.sender_id[1] = linkaddr_node_addr.u8[1];

    packetbuf_copyfrom(&pkt, sizeof(pkt));
    broadcast_send(&broadcast);

    printf("TX: Seq=%u, Node=%u.%u\n",
           pkt.seq, pkt.sender_id[0], pkt.sender_id[1]);

    /* Remove inactive neighbors */
    cleanup_neighbors(clock_time());
  }

  PROCESS_END();
}
