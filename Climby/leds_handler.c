#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <i2c_bus.h>

//#include <main.h> //COPIED FROM TP5 ; I HAVE MY DOUBTS
#include <motors.h>
#include <sensors/imu.h>
#include <sensors/proximity.h>
#include <sensors/VL53L0X/VL53L0X.h> //ToF
#include <leds.h>

#include <leds_handler.h>

void leds_handler(int state, int16_t bearing){
	clear_leds();

	switch(state){
	case 0: //Calibration state
		set_rgb_led(LED2, -1, 0, 0); //Light up red
		set_rgb_led(LED4, -1, 0, 0);
		set_rgb_led(LED6, -1, 0, 0);
		set_rgb_led(LED8, -1, 0, 0);
		break;
	case 1: //Movement state
		if(bearing <= 0)
			set_rgb_led(LED2, 0, 0, -1); //Light up blue
		if(bearing >= 0)
			set_rgb_led(LED8, 0, 0, -1);
		break;
	case 2: //Top reached state
		set_rgb_led(LED2, 0, -1, 0); //Light up green
		set_rgb_led(LED4, 0, -1, 0);
		set_rgb_led(LED6, 0, -1, 0);
		set_rgb_led(LED8, 0, -1, 0);
		break;
	}
}

void blink_leds(int state, int16_t bearing, int blink_period_cycles){
	static int prev_state = 0;

	static int blink_counter = 0;
	static int blink_state = 1;

	if(state != prev_state){
		clear_leds();
		blink_counter = 0;
		blink_state = 1;

		prev_state = state;

		leds_handler(state, bearing);
	}
	else if(blink_counter == blink_period_cycles){
		blink_counter = 0;
		blink_state = !blink_state;

		if(blink_state){
			leds_handler(state, bearing);
		}
		else
			clear_leds();
	}
	//chprintf((BaseSequentialStream *)&SD3, "Counter = %d", blink_counter);
	++blink_counter;
}
