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

#define SLEEP_CYCLE  11 - 1        	      // 0 for never sleep
#define SLEEP_SLOT WAKE_TIME   // sleep slot should not be too large to prevent overflow

// For neighbour discovery, we would like to send message to everyone. We use Broadcast address:
linkaddr_t dest_addr;

#define NUM_SEND 2

// #define MAX_DATA_POINTS 60   // Collect 60 sets of readings
#define MAX_DATA_POINTS 10   // Collect 10 sets of readings for testing

/*---------------------------------------------------------------------------*/
typedef struct {
  int light_intensity;  // Light reading
  int motion_value;     // Motion reading (aggregated values from accelerometer and gyroscope)
} sensor_data_tuple;

typedef struct {
  unsigned long src_id;
  unsigned long timestamp;
  unsigned int count;
  sensor_data_tuple data[MAX_DATA_POINTS]; // Start with MAX_DATA_POINTS number of invalid points in the array
  unsigned int data_count;  // Number of valid data points in the array
  
} data_packet_struct;

/*---------------------------------------------------------------------------*/
// duty cycle = WAKE_TIME / (WAKE_TIME + SLEEP_SLOT * SLEEP_CYCLE)
/*---------------------------------------------------------------------------*/

// sender timer implemented using rtimer
static struct rtimer rt;

// Protothread variable
static struct pt pt;

// Structure holding the data to be transmitted
static data_packet_struct data_packet;

// Current time stamp of the node
unsigned long curr_timestamp;

// Boolean to check if it is the first time new node is detected
static bool new_node_detected; 

// Starts the main contiki neighbour discovery process
PROCESS(nbr_discovery_process, "cc2650 neighbour discovery process");
AUTOSTART_PROCESSES(&nbr_discovery_process);

// Function called after reception of a packet
void receive_packet_callback(const void *data, uint16_t len, const linkaddr_t *src, const linkaddr_t *dest) 
{
  if(len == sizeof(data_packet)) {

    static data_packet_struct received_packet_data;
    
    // Copy the content of packet into the data structure
    memcpy(&received_packet_data, data, len);
    

    // Print the details of the received packet
    // printf("recv %lu rssi %d at: %3lu.%03lu\n",
    //     received_packet_data.count, (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI),
    //     received_packet_data.timestamp / CLOCK_SECOND,
    //     ((received_packet_data.timestamp % CLOCK_SECOND)*1000) / CLOCK_SECOND);

    // Print once only when new node detected  
    if (new_node_detected) {
      printf("%lu DETECT %d", received_packet_data.timestamp / CLOCK_SECOND, received_packet_data.src_id);
      new_node_detected = false;
    }

    int rssi = (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI);

    // check if there is good link quality
    if (rssi >= -70) {
      printf("Good link quality established with rssi: %d\n", rssi);
      // send pkt to node A to signal to it to start transferring stored readings

    } 
        
  }

}

// Scheduler function for the sender of neighbour discovery packets
char sender_scheduler(struct rtimer *t, void *ptr) {
 
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

       // Initialize the nullnet module with information of packet to be trasnmitted
      nullnet_buf = (uint8_t *)&data_packet; //data transmitted
      nullnet_len = sizeof(data_packet); //length of data transmitted
      
      data_packet.count++;
      
      curr_timestamp = clock_time();
      
      data_packet.timestamp = curr_timestamp;

      // printf("Send seq# %lu  @ %8lu ticks   %3lu.%03lu\n", data_packet.seq, curr_timestamp, curr_timestamp / CLOCK_SECOND, ((curr_timestamp % CLOCK_SECOND)*1000) / CLOCK_SECOND);

      NETSTACK_NETWORK.output(&dest_addr); //Packet transmission
      

      // wait for WAKE_TIME before sending the next packet
      if(i != (NUM_SEND - 1)){

        rtimer_set(t, RTIMER_TIME(t) + WAKE_TIME, 1, (rtimer_callback_t)sender_scheduler, ptr);
        PT_YIELD(&pt);
      
      }
   
    }

    // sleep for a random number of slots
    if(SLEEP_CYCLE != 0){
    
      // radio off
      NETSTACK_RADIO.off();

      // SLEEP_SLOT cannot be too large as value will overflow,
      // to have a large sleep interval, sleep many times instead

      // get a value that is uniformly distributed between 0 and 2*SLEEP_CYCLE
      // the average is SLEEP_CYCLE 
      NumSleep = SLEEP_CYCLE;  
      //printf(" Sleep for %d slots \n",NumSleep);

      // NumSleep should be a constant or static int
      rtimer_set(t, RTIMER_TIME(t) + SLEEP_SLOT * SLEEP_CYCLE, 1, (rtimer_callback_t)sender_scheduler, ptr);
      PT_YIELD(&pt);

    }
  }
  
  PT_END(&pt);
}

// Main thread that handles the neighbour discovery process
PROCESS_THREAD(nbr_discovery_process, ev, data)
{

 // static struct etimer periodic_timer;

  PROCESS_BEGIN();

    // initialize data packet sent for neighbour discovery exchange
  data_packet.src_id = node_id; //Initialize the node ID
  data_packet.count = 0; //Initialize the sequence number of the packet
  
  nullnet_set_input_callback(receive_packet_callback); //initialize receiver callback
  linkaddr_copy(&dest_addr, &linkaddr_null);

  new_node_detected = true;

  printf("CC2650 neighbour discovery\n");
  printf("Node %d will be sending packet of size %d Bytes\n", node_id, (int)sizeof(data_packet_struct));

  // Start sender in one millisecond.
  rtimer_set(&rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1, (rtimer_callback_t)sender_scheduler, NULL);

  

  PROCESS_END();
}
