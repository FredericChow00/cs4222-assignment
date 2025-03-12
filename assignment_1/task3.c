#include <stdio.h>

#include "contiki.h"
#include "sys/rtimer.h"
#include "sys/etimer.h"
#include "buzzer.h"

#include "board-peripherals.h"

#include <stdint.h>

PROCESS(process_main, "Main");
AUTOSTART_PROCESSES(&process_main);

static void wait();
static void wait_main();
static void buzz();
static void buzz_main();
static void buzz_n_wait();

static void idle_main();
static void interim();
static void interim_main();


static struct rtimer timer;


static int prev;
static rtimer_clock_t stop_after;
static bool change_happened;
static void buzz_main() {
  int val = opt_3001_sensor.value(0);
  rtimer_clock_t now = RTIMER_NOW();
  if (val != CC26XX_SENSOR_READING_ERROR) {
    val = val / 100;
    if (prev != -1 && abs(val - prev) >= 300) {
      change_happened = true;
      printf("b: light %d -> %d\n", prev, val);
    }
    prev = val;
  } else {
    printf("light sensor not ready\n");
  }

  if (now >= stop_after) {
    buzzer_stop();
    if (change_happened) {
      printf("BUZZ -> IDLE\n");
      idle_main();
    } else {
      printf("BUZZ -> WAIT\n");
      wait();
    }
  } else {
    SENSORS_ACTIVATE(opt_3001_sensor);
    rtimer_set(&timer, now + RTIMER_SECOND / 4, 0, buzz_main, NULL);
  }
}

static void wait_main() {
  int val = opt_3001_sensor.value(0);
  rtimer_clock_t now = RTIMER_NOW();
  if (val != CC26XX_SENSOR_READING_ERROR) {
    val = val / 100;
    if (prev != -1 && abs(val - prev) >= 300) {
      change_happened = true;
      printf("b: light %d -> %d\n", prev, val);
    }
    prev = val;
  } else {
    printf("light sensor not ready\n");
  }

  if (now >= stop_after) {
    printf("WAIT -> BUZZ\n");
    buzz();
  } else {
    SENSORS_ACTIVATE(opt_3001_sensor);
    rtimer_set(&timer, now + RTIMER_SECOND / 4, 0, wait_main, NULL);
  }
  // int val = opt_3001_sensor.value(0);
  // rtimer_clock_t now = RTIMER_NOW();
  // if (val == CC26XX_SENSOR_READING_ERROR || prev == -1 || abs(val - prev) < 300) {
  //   // printf("w: no %d %d %d\n", val == CC26XX_SENSOR_READING_ERROR, prev == -1, abs(val - prev) < 300);
  //   if (now > stop_after) {
  //     printf("WAIT -> BUZZ\n");
  //     buzz();
  //     return;
  //   }
  // } else {
  //   printf("w: light %d -> %d\n", prev, val);
  //   change_happened = true;
  // }
  // prev = val;
  
  // SENSORS_ACTIVATE(opt_3001_sensor);
  // rtimer_set(&timer, now + RTIMER_SECOND / 4, 0, wait_main, NULL);
}

static void buzz_n_wait() {
  change_happened = false;
  prev = -1;
  buzz();
}

static void buzz() {
  rtimer_clock_t now = RTIMER_NOW();
  stop_after = now + 2 * RTIMER_SECOND;
  
  buzzer_start(1000);
  SENSORS_ACTIVATE(opt_3001_sensor);
  rtimer_set(&timer, now + RTIMER_SECOND / 4, 0, buzz_main, NULL);
  
}

static void wait() {
  rtimer_clock_t now = RTIMER_NOW();
  stop_after = now + 4 * RTIMER_SECOND;
  
  SENSORS_ACTIVATE(opt_3001_sensor);
  rtimer_set(&timer, now + RTIMER_SECOND / 4, 0, wait_main, NULL);
}

static void idle_main() {
  int x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);

  int y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);

  int z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);

  int rot_x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X);

  int rot_y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y);

  int rot_z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z);

  int all = (int)sqrt(x*x + y*y + z*z) - 100;

  int rot_all = (int)sqrt(rot_x*rot_x + rot_y*rot_y + rot_z*rot_z);
  // printf("acc %d.%02d G\n", all / 100, abs(all) % 100);
  // printf("rot %d.%02d deg/s\n", rot_all / 100, abs(rot_all) % 100);
  if (all <= 100 && rot_all <= 20000) {
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
  if (val != CC26XX_SENSOR_READING_ERROR) {
    val = val / 100;
    if (prev != -1 && abs(val - prev) >= 300) {
      printf("b: light %d -> %d\n", prev, val);
      printf("INTERIM -> BUZZ\n");
      buzz_n_wait();
      return;
    }
    prev = val;
  } else {
    printf("light sensor not ready\n");
  }
  SENSORS_ACTIVATE(opt_3001_sensor);
  rtimer_set(&timer, now + RTIMER_SECOND / 4, 0, interim_main, NULL);
}

PROCESS_THREAD(process_main, ev, data) {
  
  PROCESS_BEGIN();
  
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
  printf("in main\n");
  idle_main();

  PROCESS_END();
}
