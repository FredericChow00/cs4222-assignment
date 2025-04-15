/*
* CS4222/5422: Assignment 3b
* Perform neighbour discovery
*/

#include "contiki.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "net/packetbuf.h"
#include "lib/random.h"
#include "net/linkaddr.h"
#include <string.h>
#include <stdio.h> 
#include "node-id.h"

// Identification information of the node


// Configures the wake-up timer for neighbour discovery 
#define WAKE_TIME RTIMER_SECOND/22    // 10 HZ, 0.1s

#define SLEEP_CYCLE  11 - 1
#define SLEEP_SLOT WAKE_TIME   // sleep slot should not be too large to prevent overflow

// For neighbour discovery, we would like to send message to everyone. We use Broadcast address:
linkaddr_t dest_addr;

#define NUM_SEND 2

// #define MAX_DATA_POINTS 60   // Collect 60 sets of readings
#define MAX_DATA_POINTS 10   // Collect 10 sets of readings for testing

/*---------------------------------------------------------------------------*/
typedef struct {
  unsigned long src_id;
  unsigned long dest_id;
  unsigned long timestamp;
  
} discovery_packet_struct;

typedef struct {
  unsigned long src_id;
  unsigned long dest_id;
  int light_data[MAX_DATA_POINTS]; // Fixed size array of MAX_DATA_POINTS
  int motion_data[MAX_DATA_POINTS]; // Fixed size array of MAX_DATA_POINTS

} data_packet_struct;

/*---------------------------------------------------------------------------*/
// duty cycle = WAKE_TIME / (WAKE_TIME + SLEEP_SLOT * SLEEP_CYCLE)
/*---------------------------------------------------------------------------*/

// sender timer implemented using rtimer
static struct rtimer rt;

// Protothread variable
static struct pt pt;

// Structure holding data for discovery process
static discovery_packet_struct discovery_pkt;

// Current time stamp of the node
unsigned long curr_timestamp;

// Whether node B should be sending ACK discovery packets
static int send_ack = 0;

// Starts the main contiki neighbour discovery process
PROCESS(nbr_discovery_process, "cc2650 neighbour discovery process");
AUTOSTART_PROCESSES(&nbr_discovery_process);

// Function called after reception of a packet
void receive_packet_callback(const void *data, uint16_t len, const linkaddr_t *src, const linkaddr_t *dest) 
{
  // Check if the received packet size matches with what we expect it to be
  if(len == sizeof(discovery_pkt)) {
    static discovery_packet_struct received_packet_data;
    
    // Copy the content of packet into the data structure
    memcpy(&received_packet_data, data, len);

    // Check if packet was a broadcast message
    if (received_packet_data.src_id != received_packet_data.dest_id) {
      return;
    }

    int rssi = (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI);
    
    // Print the details of the received packet
    printf("src: %lu, dest: %lu, rssi: %d, at time: %3lu.%03lu\n",
      received_packet_data.src_id, received_packet_data.dest_id, rssi,
      received_packet_data.timestamp / CLOCK_SECOND,
      ((received_packet_data.timestamp % CLOCK_SECOND)*1000) / CLOCK_SECOND);

    // Based on node A timestamp
    printf("%lu DETECT %d", received_packet_data.timestamp / CLOCK_SECOND, received_packet_data.src_id);

    // check if there is good link quality
    if (rssi < -70) {
      return;
    }

    printf("Good link quality established with rssi: %d\n", rssi);

    // send pkt to node A to signal to it to start transferring stored readings
    nullnet_buf = (uint8_t *)&discovery_pkt; //data transmitted
    nullnet_len = sizeof(discovery_pkt); //length of data transmitted
    discovery_pkt.dest_id = received_packet_data.src_id;
    discovery_pkt.timestamp = clock_time();
    send_ack = 1; // Keep retrying if send on line below fails
    NETSTACK_NETWORK.output(&dest_addr); //Packet transmission
        
  } else if (len == sizeof(data_packet_struct)) {
    static data_packet_struct received_packet_data;
    
    // Copy the content of packet into the data structure
    memcpy(&received_packet_data, data, len);

    // Check if packet was sent to this node
    if (received_packet_data.dest_id != node_id) {
      return;
    }
    
    send_ack = 0; // Reset to no longer send

    printf("Light: %d", received_packet_data.light_data[0]);
    for (int data_count = 1; data_count < MAX_DATA_POINTS; data_count++) {
      printf(", %d", received_packet_data.light_data[data_count]);
    }

    printf("\nMotion: %d", received_packet_data.motion_data[0]);
    for (int data_count = 1; data_count < MAX_DATA_POINTS; data_count++) {
      printf(", %d", received_packet_data.light_data[data_count]);
    }
    printf("\n");
  }

}

// Scheduler function for the listening of neighbour discovery packets
char listening_scheduler(struct rtimer *t, void *ptr) {
 
  static uint16_t i = 0;
  
  static int NumSleep=0;
 
  // Begin the protothread
  PT_BEGIN(&pt);

  // Get the current time stamp
  curr_timestamp = clock_time();

  printf("Start clock %lu ticks, timestamp %3lu.%03lu\n", curr_timestamp, curr_timestamp / CLOCK_SECOND, 
  ((curr_timestamp % CLOCK_SECOND)*1000) / CLOCK_SECOND);

  while(1){

    // radio on
    NETSTACK_RADIO.on();

    // send NUM_SEND number of neighbour discovery beacon packets
    for(i = 0; i < NUM_SEND; i++){
      // don't need to send any packets until received a discovery packet from node A
      if (send_ack) {
        NETSTACK_NETWORK.output(&dest_addr); //Packet transmission
      }

      // wait for WAKE_TIME before sending the next packet
      if(i != (NUM_SEND - 1)){
        rtimer_set(t, RTIMER_TIME(t) + WAKE_TIME, 1, (rtimer_callback_t)listening_scheduler, ptr);
        PT_YIELD(&pt);
      }
    }

    // sleep for a fixed number of slots
    if (!send_ack) { // Remain on if attempting to receive from node A
      // radio off
      NETSTACK_RADIO.off();
    }

    // (SLEEP_SLOT = 65536 / 22) * (SLEEP_CYCLE = 11 - 1) <= 2147483647 so it won't overflow
    rtimer_set(t, RTIMER_TIME(t) + SLEEP_SLOT * SLEEP_CYCLE, 1, (rtimer_callback_t)listening_scheduler, ptr);
    PT_YIELD(&pt);
    
    send_ack = 0; // Reset to no longer send (since node A cycle < node B cycle, if node B did not receive by now, node A has disconnected
  }
  
  PT_END(&pt);
}

// Main thread that handles the neighbour discovery process
PROCESS_THREAD(nbr_discovery_process, ev, data)
{

 // static struct etimer periodic_timer;

  PROCESS_BEGIN();

  // initialize data packet sent for neighbour discovery exchange
  discovery_pkt.src_id = node_id; //Initialize the node ID
  
  nullnet_set_input_callback(receive_packet_callback); //initialize receiver callback
  linkaddr_copy(&dest_addr, &linkaddr_null);

  printf("CC2650 neighbour discovery\n");
  printf("Node %d will be sending discovery packets of size %d Bytes\n", node_id, (int)sizeof(discovery_packet_struct));

  // Start sender in one millisecond.
  rtimer_set(&rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1, (rtimer_callback_t)listening_scheduler, NULL);

  PROCESS_END();
}
