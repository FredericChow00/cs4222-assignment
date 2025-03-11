#include <stdio.h>

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
static void wait();
static void wait_main();
static void buzz();
static void buzz_main();
static void buzz_n_wait();


static struct rtimer timer;



// static int prev_light;
// static void poll_sig_light_changes_main() {
//   int val = opt_3001_sensor.value(0);
//   if (val == CC26XX_SENSOR_READING_ERROR || prev_light == -1 || abs(val - prev_light) < 300) {
//     printf("no light %d %d %d\n", val == CC26XX_SENSOR_READING_ERROR, prev_light == -1, abs(val - prev_light) < 300);
//     prev_light = val;  
//     SENSORS_ACTIVATE(opt_3001_sensor);
//     rtimer_set(&timer, RTIMER_NOW() + RTIMER_SECOND, 0, helper, NULL);
//   } else {
//     printf("light %d -> %d\n", prev_light, val);
//     leave();
//   }

// }

// static void poll_sig_light_changes() {
//   printf("got to poll\n");

//   prev_light = -1;
//   SENSORS_ACTIVATE(opt_3001_sensor);
//   rtimer_set(&timer, RTIMER_NOW() + RTIMER_SECOND, 0, poll_sig_light_changes_main, NULL);
// }


static int prev;
static rtimer_clock_t stop_after;
static bool change_happened;
static void buzz_main() {
  int val = opt_3001_sensor.value(0);
  rtimer_clock_t now = RTIMER_NOW();
  if (val == CC26XX_SENSOR_READING_ERROR || prev == -1 || abs(val - prev) < 300) {
    // printf("b: no %d %d %d\n", val == CC26XX_SENSOR_READING_ERROR, prev == -1, abs(val - prev) < 300);
    if (now > stop_after) {
      buzzer_stop();
      if (change_happened) {
        printf("BUZZ -> IDLE\n");
        idle();
      } else {
        printf("BUZZ -> WAIT\n");
        wait();
      }
      return;
    }
  } else {
    printf("b: light %d -> %d\n", prev, val);
    change_happened = true;
  }
  prev = val;
  
  SENSORS_ACTIVATE(opt_3001_sensor);
  rtimer_set(&timer, now + RTIMER_SECOND / 4, 0, buzz_main, NULL);
}

static void wait_main() {
  int val = opt_3001_sensor.value(0);
  rtimer_clock_t now = RTIMER_NOW();
  if (val == CC26XX_SENSOR_READING_ERROR || prev == -1 || abs(val - prev) < 300) {
    // printf("w: no %d %d %d\n", val == CC26XX_SENSOR_READING_ERROR, prev == -1, abs(val - prev) < 300);
    if (now > stop_after) {
      printf("WAIT -> BUZZ\n");
      buzz();
      return;
    }
  } else {
    printf("w: light %d -> %d\n", prev, val);
    change_happened = true;
  }
  prev = val;
  
  SENSORS_ACTIVATE(opt_3001_sensor);
  rtimer_set(&timer, now + RTIMER_SECOND / 4, 0, wait_main, NULL);
}

// static void poll_sig_light_changes_2() {
//   printf("got to poll\n");

//   prev_light = -1;
//   SENSORS_ACTIVATE(opt_3001_sensor);
//   rtimer_set(&timer, RTIMER_NOW() + RTIMER_SECOND, 0, poll_sig_light_changes_main_2, NULL);
// }

void idle() {

}

static void buzz_n_wait() {
  change_happened = false;
  prev = -1;
  buzz();
}

static void buzz() {
  // rtimer_clock_t now = RTIMER_NOW();
  // stop_after = RTIMER_NOW() + 2 * RTIMER_SECOND;
  
  // buzzer_start(500);
  // SENSORS_ACTIVATE(opt_3001_sensor);
  // rtimer_set(&timer, RTIMER_NOW() + RTIMER_SECOND / 4, 0, buzz_main, NULL);
  
}

static void wait() {
  rtimer_clock_t now = RTIMER_NOW();
  stop_after = RTIMER_NOW() + 4 * RTIMER_SECOND;
  
  SENSORS_ACTIVATE(opt_3001_sensor);
  rtimer_set(&timer, RTIMER_NOW() + RTIMER_SECOND / 4, 0, wait_main, NULL);
}


// static void print_light_reading_interval() {
//   int res = opt_3001_sensor.value(0);
//   if (res == CC26XX_SENSOR_READING_ERROR) {
//     printf("warming up\n");
//   } else {
//     printf("res is %d\n", res);
//   }
//   SENSORS_ACTIVATE(opt_3001_sensor);
//   rtimer_set(&timer, RTIMER_NOW() + RTIMER_SECOND, 0, print_light_reading_interval, NULL);
// }

// static void my_func(struct rtimer *_timer, void *ptr) {
//   printf("Hello world 1\n");
//   rtimer_set(&timer, RTIMER_NOW() + RTIMER_SECOND, 0, my_func, NULL);
// }


PROCESS_THREAD(process_main, ev, data) {
  
  PROCESS_BEGIN();
  
  // mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
  // SENSORS_ACTIVATE(opt_3001_sensor);
  
  printf("in main\n");
  buzzer_init();
  buzz_n_wait();
  // while (1) {


  //   // rtimer_set(&timer, RTIMER_NOW() + RTIMER_SECOND, 0, my_func, NULL);
  //   // printf("mains");
  //   PROCESS_YIELD();
  // }

  PROCESS_END();
}