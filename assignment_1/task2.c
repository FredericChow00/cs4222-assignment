#include <stdio.h>

#include "contiki.h"
#include "sys/rtimer.h"
#include "sys/etimer.h"
#include "buzzer.h"

#include "board-peripherals.h"

#include <stdint.h>

PROCESS(process_main, "Main");
AUTOSTART_PROCESSES(&process_main);

static int loop_cnt;
static int counter_rtimer;
static int counter_etimer;
static struct rtimer timer_rtimer;
static struct etimer timer_etimer;
static rtimer_clock_t timeout_rtimer = RTIMER_SECOND /4;
static int prv_lux_value = -1;
static int buzzer_status = 0;
#define ACCEL_THRESHOLD 200 
#define GYRO_THRESHOLD 20000 
/*---------------------------------------------------------------------------*/
static int get_mpu_reading(void);
static void init_opt_reading(void);
static int get_light_reading(void);
static void init_mpu_reading(void);
static void schedule_rtimer(void);

/*---------------------------------------------------------------------------*/

static void
do_rtimer_timeout(struct rtimer *timer, void *ptr)
{

  rtimer_clock_t now=RTIMER_NOW();

  int s, ms1,ms2,ms3;
  s = now /RTIMER_SECOND;
  ms1 = (now% RTIMER_SECOND)*10/RTIMER_SECOND;
  ms2 = ((now% RTIMER_SECOND)*100/RTIMER_SECOND)%10;
  ms3 = ((now% RTIMER_SECOND)*1000/RTIMER_SECOND)%10;
  
  counter_rtimer++;
  printf("rtimer: %d (cnt) %d (ticks) %d.%d%d%d (sec) \n",counter_rtimer,now, s, ms1,ms2,ms3); 

  if (get_light_reading() || get_mpu_reading()) {
    process_poll(&process_main);
  } else {
    schedule_rtimer();
  }
}

static void
schedule_rtimer()
{
  rtimer_set(&timer_rtimer, RTIMER_NOW() + timeout_rtimer, 0, do_rtimer_timeout, NULL);
}

static int
get_light_reading()
{
  int value, lux_value;

  value = opt_3001_sensor.value(0);
  if (value != CC26XX_SENSOR_READING_ERROR) {
    lux_value = value / 100;
    printf("OPT: Light=%d.%02d lux\n", lux_value, value % 100);

    if (prv_lux_value != -1 && abs(lux_value - prv_lux_value) >= 300) {
      return 1;
    }
    prv_lux_value = lux_value;

  } else {
    printf("OPT: Light Sensor's Warming Up\n\n");
  }

  init_opt_reading();
  return 0;
}

static void
init_opt_reading(void)
{
  SENSORS_ACTIVATE(opt_3001_sensor);
}

static int
get_mpu_reading()
{
  int x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
  printf("MPU Acc: X= %d.%02d G\n", x/100, abs(x)%100);

  int y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
  printf("MPU Acc: Y= %d.%02d G\n", y/100, abs(y)%100);

  int z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
  printf("MPU Acc: Z= %d.%02d G\n", z/100, abs(z)%100);

  int rot_x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X);
  printf("MPU Gyro: X= %d.%02d deg/sec\n", rot_x/100, abs(rot_x)%100);

  int rot_y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y);
  printf("MPU Gyro: Y= %d.%02d deg/sec\n", rot_y/100, abs(rot_y)%100);

  int rot_z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z);
  printf("MPU Gyro: Z= %d.%02d deg/sec\n", rot_z/100, abs(rot_z)%100);

  int all = sqrt(x*x + y*y + z*z);

  int rot_all = sqrt(rot_x*rot_x + rot_y*rot_y + rot_z*rot_z);
  
  if (all > ACCEL_THRESHOLD || rot_all > GYRO_THRESHOLD) {
    return 1;
  } else {
    return 0;
  }
}

static void
init_mpu_reading(void)
{
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
}

static void
toggle_buzzing()
{
  clock_time_t t;
  int s, ms1, ms2, ms3;
  t = clock_time();
  s = t / CLOCK_SECOND;
  ms1 = (t% CLOCK_SECOND)*10/CLOCK_SECOND;
  ms2 = ((t% CLOCK_SECOND)*100/CLOCK_SECOND)%10;
  ms3 = ((t% CLOCK_SECOND)*1000/CLOCK_SECOND)%10;

  counter_etimer++;
  printf("Toggling buzzer to %d at time(E): %d (cnt) %d (ticks) %d.%d%d%d (sec) \n",!buzzer_status,counter_etimer,t,s,ms1,ms2,ms3); 
  // Toggle the buzzer
  if (buzzer_status)
    buzzer_stop();
  else
    buzzer_start(1000);

  buzzer_status = !buzzer_status;
}

PROCESS_THREAD(process_main, ev, data)
{
  PROCESS_BEGIN();

  init_mpu_reading();
  buzzer_init();

  while (1) {
    prv_lux_value = -1; // Reset prv_lux_value
    init_opt_reading(); // Resets opt reading
    schedule_rtimer(); // Restart sensors

    // Yield until polled
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_POLL);

    // Start buzzing
    for (loop_cnt = 0; loop_cnt < 9; loop_cnt++) { // Needs to be odd number. 18s according to TA
      toggle_buzzing();

      etimer_set(&timer_etimer, CLOCK_SECOND * 2);  // 2s timer
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_etimer));
    }
    toggle_buzzing(); // Stop buzzing and return without waiting
  }

  PROCESS_END();
}
