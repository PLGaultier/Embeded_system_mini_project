#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>
#include <motion.h>
#include "motors.h"
#include <epuck1x/utility/utility.h>
#include "sensors/proximity.h"
#include "leds.h"

//define in step/s
#define ORIENTATION_SPEED 300

#define PI 3.14

//value for which an obstacle is detected
#define THRESHOLD_FOR_COLLISION 200

//number of the proximity sensor located on the front of the epuck
#define PROXIMITY_SENSOR_IN_FRONT 0

#define STRAIGHT_SPEED 600

//step increment to be performed for each angle to be reached according to the corresponding letter
#define NBRE_STEP_FOR_ANG_A 47.8

//seconds to miliseconds
#define S_TO_MS 1000

#define WAITING_TIME 2000

static _Bool obstacle_detect = false;

static double temps_rotation;

void orientation_robot (int angle_rad)
{
	//turn off all the LEDs that counted the high times
	set_led(LED7,0);
	set_led(LED5,0);
	set_led(LED3,0);
	set_led(LED1,0);
	set_body_led(0);

	//allows the epuck to turn on itself
	left_motor_set_speed(ORIENTATION_SPEED);
	right_motor_set_speed(-ORIENTATION_SPEED);

	temps_rotation = (S_TO_MS*angle_rad*NBRE_STEP_FOR_ANG_A/ORIENTATION_SPEED);

	//allow the epuck to turn for the time necessary to reach the angle associated with the right letter
	chThdSleepMilliseconds(temps_rotation);

	left_motor_set_speed(0);
	right_motor_set_speed(0);

	chThdSleepMilliseconds(WAITING_TIME);
}


void orientation_robot_back(void)
{
	left_motor_set_speed(-ORIENTATION_SPEED);
	right_motor_set_speed(ORIENTATION_SPEED);

	chThdSleepMilliseconds(temps_rotation);

	left_motor_set_speed(0);
	right_motor_set_speed(0);

	chThdSleepMilliseconds(WAITING_TIME);
}


_Bool avance_to_obstacle(void)
{
	//Init for the proximity sensors
	messagebus_topic_t *prox_topic_test = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t prox_values; //mieux dans le main.h
	messagebus_topic_wait(prox_topic_test, &prox_values, sizeof(prox_values));

	if (!(obstacle_detect))
	{
		if(get_prox(PROXIMITY_SENSOR_IN_FRONT) < THRESHOLD_FOR_COLLISION)
		{
			right_motor_set_speed(STRAIGHT_SPEED);
			left_motor_set_speed(STRAIGHT_SPEED);
			return true;
		}
		else
		{
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			set_front_led(1);
			chThdSleepMilliseconds(WAITING_TIME);
			set_front_led(0);
			obstacle_detect = true;
			return true;
		}
	}
	else
	{
		//We go back to our starting position if the step counter goes to zero
		if ((left_motor_get_pos() <= 0) || (right_motor_get_pos() <=0))
		{
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			obstacle_detect = false;
			return false;
		}
		else
		{
			right_motor_set_speed(-STRAIGHT_SPEED);
			left_motor_set_speed(-STRAIGHT_SPEED);
			return true;
		}
	}
}

