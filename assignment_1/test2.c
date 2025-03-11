#include <stdio.h>
#include <math.h>

#include "contiki.h"
#include "sys/rtimer.h"
#include "sys/etimer.h"
#include "buzzer.h"

#include "board-peripherals.h"

#include <stdint.h>

PROCESS(process_main, "Main");
AUTOSTART_PROCESSES(&process_main);

// static void poll_sig_light_changes_main();
// static void poll_sig_light_changes();
// static void leave();
static void idle();
static void idle_main();
static void interim();
static void interim_main();
static void buzz();


static struct rtimer timer;

static void idle_main() {
  int x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);

  int y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);

  int z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);

  double all = sqrt(x*x + y*y + z*z);
  // printf("%d %d %d %f\n", x,y,x,all);
  if (all <= 200.0) {
    rtimer_set(&timer, RTIMER_NOW() + RTIMER_SECOND / 4, 0, idle_main, NULL);
  } else {
    printf("IDLE -> INTERIM\n");
    interim();
  }
}

static int prev;
static void interim() {
  prev = -1;

  SENSORS_ACTIVATE(opt_3001_sensor);
  rtimer_set(&timer, RTIMER_NOW() + RTIMER_SECOND / 4, 0, interim_main, NULL);

}

static void interim_main() {
  int val = opt_3001_sensor.value(0);
  rtimer_clock_t now = RTIMER_NOW();
  if (val == CC26XX_SENSOR_READING_ERROR || prev == -1 || abs(val - prev) < 300) {
    // printf("w: no %d %d %d\n", val == CC26XX_SENSOR_READING_ERROR, prev == -1, abs(val - prev) < 300);
    prev = val;
    SENSORS_ACTIVATE(opt_3001_sensor);
    rtimer_set(&timer, now + RTIMER_SECOND / 4, 0, interim_main, NULL);
  } else {
    printf("w: light %d -> %d\n", prev, val);
    printf("INTERIM -> BUZZ\n");
    buzz();
  }
}

static void buzz() {

}

PROCESS_THREAD(process_main, ev, data) {
  
  PROCESS_BEGIN();
  
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
  // SENSORS_ACTIVATE(opt_3001_sensor);
  
  printf("in main\n");
  idle_main();
  // while (1) {


  //   // rtimer_set(&timer, RTIMER_NOW() + RTIMER_SECOND, 0, my_func, NULL);
  //   // printf("mains");
  //   PROCESS_YIELD();
  // }

  PROCESS_END();
}