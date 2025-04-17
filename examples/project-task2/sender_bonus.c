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

#define MAX_DATA_POINTS 12   // Collect 10 sets of readings
#define SEND_REPEATS 5

// For neighbour discovery, we would like to send message to everyone. We use Broadcast address:
linkaddr_t dest_addr;

#define NUM_SEND 2

#define MOTION_THRESHOLD 130

#define SIGNIFICANT_MOTION 110
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

/*---------------------------------------------------------------------------*/
// duty cycle = WAKE_TIME / (WAKE_TIME + SLEEP_SLOT * SLEEP_CYCLE)
/*---------------------------------------------------------------------------*/

// sender timer implemented using rtimer
static struct rtimer rt;
static struct etimer data_collection_timer;
static struct etimer stationary_timer;

// Protothread variable
static struct pt pt;
static struct pt pt_rcv;

// Structure holding the data to be transmitted
static data_packet_struct data_packet;

// Structure holding data for discovery process
static discovery_packet_struct discovery_packet;

// Current time stamp of the node
static clock_time_t curr_timestamp;

// Whether this data packet has been sent
static int data_packet_sent;

// Variables to ensure collection of sensor readings only occur after receive node is stationary for a minute
static bool not_stationary_for_a_min = true;
static int stationary_secs = 0;

// Function prototypes
static uint16_t get_light_reading(void);
static void init_opt_reading(void);
static uint16_t get_motion_reading(void);
static void init_mpu_reading(void);
char receive_packet_callback(const void*, uint16_t, const linkaddr_t*, const linkaddr_t*);

// Starts the main contiki neighbour discovery process
PROCESS(nbr_discovery_process, "cc2650 neighbour discovery process");
PROCESS(data_collection_process, "Sensor data collection process");

AUTOSTART_PROCESSES(&data_collection_process);

void receive_packet_return(struct rtimer *t, void *ptr) {
  PT_SCHEDULE(receive_packet_callback);
}

// Function called after reception of a packet
char receive_packet_callback(const void *data, uint16_t len, const linkaddr_t *src, const linkaddr_t *dest) 
{
  static discovery_packet_struct received_packet_data;
  static int rssi;
  static int repeat;
  static int count;

  // Begin the protothread
  PT_BEGIN(&pt_rcv);

  // Check if the received packet size matches with what we expect it to be
  if(len == sizeof(discovery_packet) && data_packet_sent == 0) {    
    // Copy the content of packet into the data structure
    memcpy(&received_packet_data, data, len);

    // Check if packet was sent to this node
    if (received_packet_data.dest_id == node_id) {
      rssi = (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI);

      // Print the details of the received packet
      printf("src: %u, dest: %u, rssi: %d, at time: %3lu.%03lu\n",
          received_packet_data.src_id, received_packet_data.dest_id, rssi,
          received_packet_data.timestamp / CLOCK_SECOND,
          ((received_packet_data.timestamp % CLOCK_SECOND)*1000) / CLOCK_SECOND);
      
      // check if there is good link quality
      if (rssi >= -70) {
        rt.func = receive_packet_return; // Stop callback to sender_scheduler()
        nullnet_set_input_callback(NULL); // Stop receiving new packets
        rtimer_run_next(); // Run callback to ensure no interruption from rtimer
        // PT_EXIT(&pt); // Restart protothread
        data_packet_sent = 1;

        // // Get the current time stamp
        // curr_timestamp = clock_time();

        // Based on node A timestamp
        printf("%lu DETECT %d\n", (clock_time() - curr_timestamp) / CLOCK_SECOND, received_packet_data.src_id);

        // Send ACK packet to node B
        discovery_packet.dest_id = received_packet_data.src_id;
        discovery_packet.timestamp = curr_timestamp;
        nullnet_buf = (uint8_t *)&discovery_packet; //data transmitted
        nullnet_len = sizeof(discovery_packet); //length of data transmitted
        NETSTACK_NETWORK.output(src);
        discovery_packet.dest_id = node_id; // Reset to broadcast

        // rtimer_set(&rt, RTIMER_NOW() + WAKE_TIME, 1, (rtimer_callback_t)receive_packet_callback, NULL);
        // PT_YIELD(&pt_rcv);

        // Send data_packet to node B
        printf("%lu TRANSFER %lu RSSI: %d\n", (clock_time() - curr_timestamp) / CLOCK_SECOND, received_packet_data.src_id, rssi);
        data_packet.dest_id = received_packet_data.src_id;
        nullnet_buf = (uint8_t *)&data_packet; //data transmitted
        nullnet_len = sizeof(data_packet); //length of data transmitted

        //Packet transmissions, split into SEND_REPEATS number of sends
        for (repeat = 0; repeat < SEND_REPEATS; repeat++) {
          for (count = 0; count < MAX_DATA_POINTS; count++) {
            data_packet.light_data[count] = light_data[repeat * MAX_DATA_POINTS + count];
            data_packet.motion_data[count] = motion_data[repeat * MAX_DATA_POINTS + count];
          }

          NETSTACK_NETWORK.output(&dest_addr);
          // rtimer_set(&rt, RTIMER_NOW() + WAKE_TIME, 1, (rtimer_callback_t)receive_packet_callback, NULL);
          // PT_YIELD(&pt_rcv);
        }
        NETSTACK_RADIO.off();
        process_poll(&data_collection_process);
      }
    }
  }
  PT_END(&pt_rcv);
}

// Scheduler function for the sender of neighbour discovery packets
char sender_scheduler(struct rtimer *t, void *ptr) {
 
  static uint16_t i = 0;
  
  // Begin the protothread
  PT_BEGIN(&pt);

  // Get the current time stamp
  // curr_timestamp = clock_time();

  // printf("Start clock %lu ticks, timestamp %3lu.%03lu\n", curr_timestamp, curr_timestamp / CLOCK_SECOND, 
  // ((curr_timestamp % CLOCK_SECOND)*1000) / CLOCK_SECOND);

  while(1){
    // radio on
    NETSTACK_RADIO.on();

    // send NUM_SEND number of neighbour discovery beacon packets
    for(i = 0; i < NUM_SEND; i++){

      // Initialize the nullnet module with information of packet to be transmitted
      nullnet_buf = (uint8_t *)&discovery_packet; //data transmitted
      nullnet_len = sizeof(discovery_packet); //length of data transmitted

      // curr_timestamp = clock_time();
      
      discovery_packet.timestamp = clock_time();

      NETSTACK_NETWORK.output(&dest_addr); //Packet transmission

      // wait for WAKE_TIME before sending the next packet
      if(i != (NUM_SEND - 1)){
        rtimer_set(&rt, RTIMER_TIME(t) + WAKE_TIME, 1, (rtimer_callback_t)sender_scheduler, NULL);
        PT_YIELD(&pt);
      }
    }

    // sleep for a fixed number of slots
    // radio off
    NETSTACK_RADIO.off();

    // (SLEEP_SLOT = 65536 / 22) * (SLEEP_CYCLE = 9 - 1) <= 2147483647 so it won't overflow
    rtimer_set(&rt, RTIMER_TIME(t) + SLEEP_SLOT * SLEEP_CYCLE, 1, (rtimer_callback_t)sender_scheduler, NULL);
    PT_YIELD(&pt);
  }

  PT_END(&pt);
}

// Sensor reading functions
static uint16_t get_light_reading() {
  int value, lux_value;

  value = opt_3001_sensor.value(0);
  if (value != CC26XX_SENSOR_READING_ERROR) {
    lux_value = value / 100;
    // printf("Light reading: %d.%02d lux\n", lux_value, value % 100);
  } else {
    // printf("Light Sensor's Warming Up\n");
  }

  init_opt_reading();

  return lux_value;
}

static void init_opt_reading(void) {
  SENSORS_ACTIVATE(opt_3001_sensor);
}

static uint16_t get_motion_reading() {
  int x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
  int y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
  int z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
  
  // int rot_x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X);
  // int rot_y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y);
  // int rot_z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z);

  // Calculate magnitude of acceleration and rotation
  int accel_magnitude = sqrt(x*x + y*y + z*z);
  // int gyro_magnitude = sqrt(rot_x*rot_x + rot_y*rot_y + rot_z*rot_z);
  
  // Aggregate accel and gyro magnutude into a single value
  // int motion_value = accel_magnitude + (gyro_magnitude / 100);
  
  // printf("Motion reading: Accel=%d, Gyro=%d, Combined=%d\n", 
  //   accel_magnitude, gyro_magnitude, motion_value);

  // printf("Motion reading: Accel=%d\n", 
    // accel_magnitude);
    
  return accel_magnitude;
}

static void init_mpu_reading(void) {
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
}

// Data collection process
PROCESS_THREAD(data_collection_process, ev, data) {
  static int data_count;
  static uint16_t motion;
  static uint16_t light;

  PROCESS_BEGIN();

  init_mpu_reading();


  discovery_packet.src_id = node_id; //Initialize the node ID
  discovery_packet.dest_id = node_id; // Same as src_id to indicate as broadcast message
  data_packet.src_id = node_id; //Initialize the node ID
  
  linkaddr_copy(&dest_addr, &linkaddr_null);
  NETSTACK_RADIO.off();

  while (1) {
    // Initialize sensors
    init_opt_reading();
    init_mpu_reading();
    data_count = 0;

    motion = 0;

    // Block here until motion > MOTION_THRESHOLD
    printf("Waiting for significant motion...");
    while (motion < MOTION_THRESHOLD) {
      etimer_set(&data_collection_timer, CLOCK_SECOND / 4);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&data_collection_timer));
      motion = get_motion_reading();
    };

    // Collect data points at 1 second intervals
    while(data_count < SEND_REPEATS * MAX_DATA_POINTS) {
      etimer_set(&data_collection_timer, CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&data_collection_timer));
      
      // Get sensor readings
      light = get_light_reading();
      motion = get_motion_reading();
      
      // Store in the data packet
      light_data[data_count] = light;
      motion_data[data_count] = motion;
      
      printf("Collected data point %d: Light=%d, Motion=%d\n", data_count, light, motion);
        
      data_count++;
    }
    
    data_packet_sent = 0;
    stationary_secs = 0;
    
    while (not_stationary_for_a_min) {
      etimer_set(&stationary_timer, CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&stationary_timer));

      uint16_t motion = get_motion_reading();

      if (motion < SIGNIFICANT_MOTION) {
        printf("stationary for %d s\n", stationary_secs+1);
        stationary_secs ++;
        if (stationary_secs == MINUTE) {
          break;
        }
      } else {
        printf("movement detected, restart\n");
        stationary_secs = 0;
      }
    }

    curr_timestamp = clock_time();

    // Start the neighbor discovery process
    printf("CC2650 neighbour discovery started at %lu\n", curr_timestamp);
    printf("Node %d will be sending discovery packets of size %d Bytes\n", node_id, (int)sizeof(discovery_packet_struct));

    //initialize receiver callback
    nullnet_set_input_callback((void(*)(const void*, uint16_t, const linkaddr_t*, const linkaddr_t*))receive_packet_callback); // Typecast to fix mac issue

    // Start sender in one millisecond.
    rtimer_set(&rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1, (rtimer_callback_t)sender_scheduler, NULL);
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_POLL);
  }

  PROCESS_END();
}
