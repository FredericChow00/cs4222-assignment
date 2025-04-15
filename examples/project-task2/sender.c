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

#define SLEEP_CYCLE  9 - 1
#define SLEEP_SLOT WAKE_TIME   // sleep slot should not be too large to prevent overflow

// #define MAX_DATA_POINTS 60   // Collect 60 sets of readings
#define MAX_DATA_POINTS 10  // Collect 10 sets of readings for testing

// For neighbour discovery, we would like to send message to everyone. We use Broadcast address:
linkaddr_t dest_addr;

#define NUM_SEND 2
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
static struct etimer data_collection_timer;

// Protothread variable
static struct pt pt;

// Structure holding the data to be transmitted
static data_packet_struct data_packet;

// Structure holding data for discovery process
static discovery_packet_struct discovery_pkt;

// Current time stamp of the node
unsigned long curr_timestamp;

// Function prototypes for sensor reading
static int get_light_reading(void);
static void init_opt_reading(void);
static int get_motion_reading(void);
static void init_mpu_reading(void);

// Starts the main contiki neighbour discovery process
PROCESS(nbr_discovery_process, "cc2650 neighbour discovery process");
PROCESS(data_collection_process, "Sensor data collection process");

AUTOSTART_PROCESSES(&data_collection_process);

// Function called after reception of a packet
void receive_packet_callback(const void *data, uint16_t len, const linkaddr_t *src, const linkaddr_t *dest) 
{
  // Check if the received packet size matches with what we expect it to be
  if(len == sizeof(discovery_pkt)) {
    static discovery_packet_struct received_packet_data;
    
    // Copy the content of packet into the data structure
    memcpy(&received_packet_data, data, len);

    // Check if packet was sent to this node
    if (received_packet_data.dest_id != node_id) {
      return;
    }

    int rssi = (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI);

    // Print the details of the received packet
    printf("src: %lu, dest: %lu, rssi: %d, at time: %3lu.%03lu\n",
        received_packet_data.src_id, received_packet_data.dest_id, rssi,
        received_packet_data.timestamp / CLOCK_SECOND,
        ((received_packet_data.timestamp % CLOCK_SECOND)*1000) / CLOCK_SECOND);
    
    // check if there is good link quality
    if (rssi < -70) {
      return;
    }

    printf("Good link quality established with rssi: %d\n", rssi);
    
    // Get the current time stamp
    curr_timestamp = clock_time();

    // Based on node A timestamp
    printf("%lu DETECT %d", curr_timestamp / CLOCK_SECOND, received_packet_data.src_id);

    // Send data_packet to node B
    data_packet.dest_id = received_packet_data.src_id;
    nullnet_buf = (uint8_t *)&data_packet; //data transmitted
    nullnet_len = sizeof(data_packet); //length of data transmitted
    printf("%lu TRANSFER %lu RSSI: %d", curr_timestamp, data_packet.dest_id, rssi);
    NETSTACK_NETWORK.output(&dest_addr); //Packet transmission
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

      // Initialize the nullnet module with information of packet to be transmitted
      nullnet_buf = (uint8_t *)&discovery_pkt; //data transmitted
      nullnet_len = sizeof(discovery_pkt); //length of data transmitted

      curr_timestamp = clock_time();
      
      discovery_pkt.timestamp = curr_timestamp;

      NETSTACK_NETWORK.output(&dest_addr); //Packet transmission

      // wait for WAKE_TIME before sending the next packet
      if(i != (NUM_SEND - 1)){
        rtimer_set(t, RTIMER_TIME(t) + WAKE_TIME, 1, (rtimer_callback_t)sender_scheduler, ptr);
        PT_YIELD(&pt);
      }
    }

    // sleep for a fixed number of slots
    // radio off
    NETSTACK_RADIO.off();

    // (SLEEP_SLOT = 65536 / 22) * (SLEEP_CYCLE = 9 - 1) <= 2147483647 so it won't overflow
    rtimer_set(t, RTIMER_TIME(t) + SLEEP_SLOT * SLEEP_CYCLE, 1, (rtimer_callback_t)sender_scheduler, ptr);
    PT_YIELD(&pt);
  }
  
  PT_END(&pt);
}

// Sensor reading functions
static int get_light_reading() {
  int value, lux_value;

  value = opt_3001_sensor.value(0);
  if (value != CC26XX_SENSOR_READING_ERROR) {
    lux_value = value / 100;
    printf("Light reading: %d.%02d lux\n", lux_value, value % 100);
  } else {
    printf("Light Sensor's Warming Up\n");
  }

  init_opt_reading();

  return lux_value;
}

static void init_opt_reading(void) {
  SENSORS_ACTIVATE(opt_3001_sensor);
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

// Data collection process
PROCESS_THREAD(data_collection_process, ev, data) {
  static int data_count = 0;

  PROCESS_BEGIN();
  
  // Initialize sensors
  init_opt_reading();
  init_mpu_reading();
  
  printf("Starting data collection: %d readings at 1 per second\n", MAX_DATA_POINTS);
  
  // Collect data points at 1 second intervals
  while(data_count < MAX_DATA_POINTS) {
    etimer_set(&data_collection_timer, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&data_collection_timer));
    
    // Get sensor readings
    int light = get_light_reading();
    int motion = get_motion_reading();
    
    // Store in the data packet
    data_packet.light_data[data_count] = light;
    data_packet.motion_data[data_count] = motion;
    
    printf("Collected data point %d: Light=%d, Motion=%d\n", data_count, light, motion);
      
    data_count++;
  }
  
  printf("Data collection complete! Collected %u data points\n", data_count);
  
  // Start the neighbor discovery process
  process_start(&nbr_discovery_process, NULL);
  
  PROCESS_END();
}

// Main thread that handles the neighbour discovery process
PROCESS_THREAD(nbr_discovery_process, ev, data)
{

 // static struct etimer periodic_timer;

  PROCESS_BEGIN();

  discovery_pkt.src_id = node_id; //Initialize the node ID
  discovery_pkt.dest_id = node_id; // Same as src_id to indicate as broadcast message
  data_packet.src_id = node_id; //Initialize the node ID
  
  nullnet_set_input_callback(receive_packet_callback); //initialize receiver callback
  linkaddr_copy(&dest_addr, &linkaddr_null);

  printf("CC2650 neighbour discovery\n");
  printf("Node %d will be sending discovery packets of size %d Bytes\n", node_id, (int)sizeof(discovery_packet_struct));

  // Start sender in one millisecond.
  rtimer_set(&rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1, (rtimer_callback_t)sender_scheduler, NULL);

  PROCESS_END();
}
