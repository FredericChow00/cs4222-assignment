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
#include "board-peripherals.h"
#include <math.h>

// Identification information of the node


// Configures the wake-up timer for neighbour discovery 
#define WAKE_TIME RTIMER_SECOND/22    // 10 HZ, 0.1s

#define SLEEP_CYCLE  11 - 1
#define SLEEP_SLOT WAKE_TIME   // sleep slot should not be too large to prevent overflow

// For neighbour discovery, we would like to send message to everyone. We use Broadcast address:
linkaddr_t dest_addr;
linkaddr_t ack_dest_addr;

#define NUM_SEND 2

#define MAX_DATA_POINTS 12   // Collect 10 sets of readings
#define SEND_REPEATS 5

#define SIGNIFICANT_MOTION 150
#define MINUTE 60

/*---------------------------------------------------------------------------*/
typedef struct {
  uint16_t src_id; // uint16_t according to Contiki source code
  uint16_t dest_id;
  clock_time_t timestamp;
  
} discovery_packet_struct;

typedef struct {
  uint16_t src_id;
  uint16_t dest_id;
  uint16_t light_data[MAX_DATA_POINTS]; // Fixed size array of MAX_DATA_POINTS
  uint16_t motion_data[MAX_DATA_POINTS]; // Fixed size array of MAX_DATA_POINTS

} data_packet_struct;

static int light_data[SEND_REPEATS * MAX_DATA_POINTS];
static int motion_data[SEND_REPEATS * MAX_DATA_POINTS];
static int data_idx;

/*---------------------------------------------------------------------------*/
// duty cycle = WAKE_TIME / (WAKE_TIME + SLEEP_SLOT * SLEEP_CYCLE)
/*---------------------------------------------------------------------------*/

// sender timer implemented using rtimer
static struct rtimer rt;
static struct etimer stationary_timer;

// Protothread variable
static struct pt pt;
static struct pt pt_rcv;

// Structure holding data for discovery process
static discovery_packet_struct discovery_pkt;

// Current time stamp of the node
static clock_time_t curr_timestamp;

// Number of ACK discovery packets send attempts left
static int ack_num_tries_left = 0;

// Variables to ensure collection of sensor readings only occur after receive node is stationary for a minute
static bool not_stationary_for_a_min = true;
static int stationary_secs = 0;

// Function prototypes
static int get_motion_reading(void);
static void init_mpu_reading(void);
char receive_packet_callback(const void*, uint16_t, const linkaddr_t*, const linkaddr_t*);
char listening_scheduler(struct rtimer*, void *);

// Starts the main contiki neighbour discovery process
PROCESS(nbr_discovery_process, "cc2650 neighbour discovery process");
AUTOSTART_PROCESSES(&nbr_discovery_process);

void receive_packet_return(struct rtimer *t, void *ptr) {
  PT_SCHEDULE(receive_packet_callback);
}

// Function called after reception of a packet
char receive_packet_callback(const void *data, uint16_t len, const linkaddr_t *src, const linkaddr_t *dest) 
{
  static discovery_packet_struct received_discovery;
  static data_packet_struct received_data;
  static int rssi;
  static int count;
  static int data_count;

  // Begin the protothread
  PT_BEGIN(&pt_rcv);

  // Check if the received packet size matches with what we expect it to be
  if(len == sizeof(discovery_pkt)) { //12
    // Copy the content of packet into the data structure
    memcpy(&received_discovery, data, len);

    rssi = (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI);

    // Print the details of the received packet
    printf("src: %u, dest: %u, rssi: %d, at time: %3lu.%03lu\n",
      received_discovery.src_id, received_discovery.dest_id, rssi,
      received_discovery.timestamp / CLOCK_SECOND,
      ((received_discovery.timestamp % CLOCK_SECOND)*1000) / CLOCK_SECOND);
    
    // printf("ACK LEFT: %d\n", ack_num_tries_left);

    // Packet was a broadcast message, and no ACK tries left
    if (received_discovery.src_id == received_discovery.dest_id && ack_num_tries_left <= 0) { 
      // Based on node A timestamp
      printf("%lu DETECT %d\n", received_discovery.timestamp / CLOCK_SECOND, received_discovery.src_id);

      // check if there is good link quality
      if (rssi >= -70) {
        // Send pkt to node A to signal to it to start transferring stored readings
        nullnet_buf = (uint8_t *)&discovery_pkt; //data transmitted
        nullnet_len = sizeof(discovery_pkt); //length of data transmitted
        discovery_pkt.dest_id = received_discovery.src_id;
        discovery_pkt.timestamp = clock_time();
        linkaddr_copy(&ack_dest_addr, src);
        ack_num_tries_left = SLEEP_CYCLE; // If node B did not receive within SLEEP_CYCLE, node A has disconnected

        rtimer_set(&rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1, (rtimer_callback_t)listening_scheduler, NULL); 
      }

    } else if (received_discovery.dest_id == node_id) { // Received ACK packet
      ack_num_tries_left = 0; // Reset to no longer send
      data_idx = 0;

      rt.func = receive_packet_return; // Stop callback to sender_scheduler()
      rtimer_run_next(); // Run callback to ensure no interruption from rtimer
      rtimer_set(&rt, RTIMER_NOW() + RTIMER_SECOND * 1.5, 1, (rtimer_callback_t)listening_scheduler, NULL); // Sufficient time to receive data packet
    }

  } else if (len == sizeof(data_packet_struct)) {    
    // Copy the content of packet into the data structure
    memcpy(&received_data, data, len);

    // Check if packet was sent to this node
    if (received_data.dest_id == node_id) {

      // printf("Received %d data points\n", MAX_DATA_POINTS);

      //Packets received, split into SEND_REPEATS number of sends
      for (count = 0; count < MAX_DATA_POINTS; count++) {
        light_data[data_idx] = received_data.light_data[count];
        motion_data[data_idx++] = received_data.motion_data[count];
      }

      if (data_idx == SEND_REPEATS * MAX_DATA_POINTS) { // Received all packets
        printf("Light: %u", light_data[0]);
        for (data_count = 1; data_count < SEND_REPEATS * MAX_DATA_POINTS; data_count++) {
          printf(", %u", light_data[data_count]);
        }
    
        printf("\nMotion: %u", motion_data[0]);
        for (data_count = 1; data_count < SEND_REPEATS * MAX_DATA_POINTS; data_count++) {
          printf(", %u", motion_data[data_count]);
        }
        printf("\n");

        rtimer_set(&rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1, (rtimer_callback_t)listening_scheduler, NULL); 
      }

      // printf("OUT\n");
    }
  }
  PT_END(&pt_rcv);
}

// Scheduler function for the listening of neighbour discovery packets
char listening_scheduler(struct rtimer *t, void *ptr) {
 
  static uint16_t i = 0;
  
  // Begin the protothread
  PT_BEGIN(&pt);

  // Get the current time stamp
  curr_timestamp = clock_time();

  // printf("Start clock %lu ticks, timestamp %3lu.%03lu\n", curr_timestamp, curr_timestamp / CLOCK_SECOND, 
  // ((curr_timestamp % CLOCK_SECOND)*1000) / CLOCK_SECOND);

  while(1){
    // printf("LISTENING");
    if (ack_num_tries_left > 0) { // Send pkt to node A to signal to it to start transferring stored readings
      ack_num_tries_left--;
      NETSTACK_NETWORK.output(&ack_dest_addr); //Packet transmission
  
      rtimer_set(&rt, RTIMER_TIME(t) + WAKE_TIME, 1, (rtimer_callback_t)listening_scheduler, NULL);
      PT_YIELD(&pt);

    } else { // Don't need to send any packets until received a discovery packet from node A
      NETSTACK_RADIO.off();
      // (SLEEP_SLOT = 65536 / 22) * (SLEEP_CYCLE = 11 - 1) <= 2147483647 so it won't overflow
      rtimer_set(&rt, RTIMER_TIME(t) + SLEEP_SLOT * SLEEP_CYCLE, 1, (rtimer_callback_t)listening_scheduler, NULL);
      PT_YIELD(&pt);

      NETSTACK_RADIO.on(); // on() is at bottom to ensure radio is on for the next loop
      rtimer_set(&rt, RTIMER_TIME(t) + WAKE_TIME, 1, (rtimer_callback_t)listening_scheduler, NULL);
      PT_YIELD(&pt);
    }
  }
  
  PT_END(&pt);
}

static int get_motion_reading() {
  int x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
  int y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
  int z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
  
  int rot_x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X);
  int rot_y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y);
  int rot_z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z);

  // Calculate magnitude of acceleration and rotation
  int accel_magnitude = sqrt(x*x + y*y + z*z);
  int gyro_magnitude = sqrt(rot_x*rot_x + rot_y*rot_y + rot_z*rot_z);
  
  // Aggregate accel and gyro magnutude into a single value
  int motion_value = accel_magnitude + (gyro_magnitude / 100);
  
  printf("Motion reading: Accel=%d, Gyro=%d, Combined=%d\n", 
    accel_magnitude, gyro_magnitude, motion_value);
    
  return motion_value;
}

static void init_mpu_reading(void) {
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
}

// Main thread that handles the neighbour discovery process
PROCESS_THREAD(nbr_discovery_process, ev, data) {

  PROCESS_BEGIN();

  init_mpu_reading();

  // while (not_stationary_for_a_min) {
  //   etimer_set(&stationary_timer, CLOCK_SECOND);
  //   PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&stationary_timer));

  //   int motion = get_motion_reading();

  //   if (motion < SIGNIFICANT_MOTION) {
  //     printf("stationary for %d s\n", stationary_secs+1);
  //     stationary_secs ++;
  //     if (stationary_secs == MINUTE) {
  //       not_stationary_for_a_min = false;
  //     }
  //   } else {
  //     printf("movement detected, restart\n");
  //     stationary_secs = 0;
  //   }
  // }

  // initialize data packet sent for neighbour discovery exchange
  discovery_pkt.src_id = node_id; //Initialize the node ID
  
  nullnet_set_input_callback(receive_packet_callback); //initialize receiver callback
  linkaddr_copy(&dest_addr, &linkaddr_null);

  printf("CC2650 neighbour discovery\n");
  printf("Node %d will be receiving discovery packets of size %d Bytes\n", node_id, (int)sizeof(discovery_packet_struct));

  // Start sender in one millisecond.
  rtimer_set(&rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1, (rtimer_callback_t)listening_scheduler, NULL);

  PROCESS_END();
}
