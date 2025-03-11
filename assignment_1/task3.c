/*
* Copyright (C) 2015, Intel Corporation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
* OF THE POSSIBILITY OF SUCH DAMAGE.
*/
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
  if (val == CC26XX_SENSOR_READING_ERROR || prev == -1 || abs(val - prev) < 300) {
    // printf("b: no %d %d %d\n", val == CC26XX_SENSOR_READING_ERROR, prev == -1, abs(val - prev) < 300);
    if (now > stop_after) {
      buzzer_stop();
      if (change_happened) {
        printf("BUZZ -> IDLE\n");
        idle_main();
      } else {
        printf("BUZZ -> WAIT\n");
        wait();
      }
      return;
    }
  } else {
    // printf("b: light %d -> %d\n", prev, val);
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
    // printf("w: light %d -> %d\n", prev, val);
    change_happened = true;
  }
  prev = val;
  
  SENSORS_ACTIVATE(opt_3001_sensor);
  rtimer_set(&timer, now + RTIMER_SECOND / 4, 0, wait_main, NULL);
}

static void buzz_n_wait() {
  change_happened = false;
  prev = -1;
  buzz();
}

static void buzz() {
  rtimer_clock_t now = RTIMER_NOW();
  stop_after = RTIMER_NOW() + 2 * RTIMER_SECOND;
  
  buzzer_start(1000);
  SENSORS_ACTIVATE(opt_3001_sensor);
  rtimer_set(&timer, RTIMER_NOW() + RTIMER_SECOND / 4, 0, buzz_main, NULL);
  
}

static void wait() {
  rtimer_clock_t now = RTIMER_NOW();
  stop_after = RTIMER_NOW() + 4 * RTIMER_SECOND;
  
  SENSORS_ACTIVATE(opt_3001_sensor);
  rtimer_set(&timer, RTIMER_NOW() + RTIMER_SECOND / 4, 0, wait_main, NULL);
}

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
    // printf("w: light %d -> %d\n", prev, val);
    printf("INTERIM -> BUZZ\n");
    buzz_n_wait();
  }
}

PROCESS_THREAD(process_main, ev, data) {
  
  PROCESS_BEGIN();
  
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
  printf("in main\n");
  idle_main();

  PROCESS_END();
}
