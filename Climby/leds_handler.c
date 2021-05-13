#include "ch.h"
#include "hal.h"
//#include <math.h>
//#include <usbcfg.h>
#include <chprintf.h>
//#include <i2c_bus.h>

#include <leds.h>

#include <leds_handler.h>

void toggle_calibration_leds(void){
	static bool leds_state = 1; //Turn on leds on first cycle
	if(leds_state)
		clear_leds();
	else{
		set_rgb_led(LED2, INTENSITY_MAX, INTENSITY_MED, 0); //Orange
		set_rgb_led(LED4, INTENSITY_MAX, INTENSITY_MED, 0);
		set_rgb_led(LED6, INTENSITY_MAX, INTENSITY_MED, 0);
		set_rgb_led(LED8, INTENSITY_MAX, INTENSITY_MED, 0);
	}
	leds_state = !leds_state;
}

void set_movement_leds(int8_t rotation){
	if(rotation < 0){
		set_rgb_led(LED2, 0, 0, INTENSITY_MAX); //Blue
		set_rgb_led(LED8, 0, 0, 0);
		return;
	}
	if(rotation == 0){
		set_rgb_led(LED2, 0, 0, INTENSITY_MAX);
		set_rgb_led(LED8, 0, 0, INTENSITY_MAX);
		return;
	}
	set_rgb_led(LED2, 0, 0, 0);
	set_rgb_led(LED8, 0, 0, INTENSITY_MAX);
}

void toggle_topreached_leds(void){
	static int counter = TOPREACHED_COUNTER_MAX; //Turn on leds on first cycle
	if(counter >= TOPREACHED_COUNTER_MAX){
		counter = 0;
		toggle_rgb_led(LED2, GREEN_LED, INTENSITY_MAX);
		toggle_rgb_led(LED4, GREEN_LED, INTENSITY_MAX);
		toggle_rgb_led(LED6, GREEN_LED, INTENSITY_MAX);
		toggle_rgb_led(LED8, GREEN_LED, INTENSITY_MAX);
	}
	++counter;
}

void climby_leds_handler(leds_state_t state, int8_t rotation){
	static leds_state_t prev_state = 0;
	if(prev_state != state)
		clear_leds();
	switch(state){
	case CALIBRATION:
		toggle_calibration_leds();
		break;
	case MOVEMENT:
		set_movement_leds(rotation);
		break;
	case TOP_REACHED:
		toggle_topreached_leds();
		break;
	}
}

//OLDER FUNCTIONS I REMOVED :

/*
void climby_leds_handler(leds_state_t state, int16_t bearing){
	static leds_state_t prev_state = 0;
	static int8_t blink_counter = 0;
	if(state != prev_state){
		blink_counter = 0;
		clear_leds();
		if(state == MOVEMENT)
			set_movement_leds(bearing);
		else
			toggle_blinking_leds(state);
	}
	else if(state == MOVEMENT){
		set_movement_leds(bearing);
	}
	else{
		if(blink_counter >= BLINK_COUNTER_MAX){
			blink_counter = 0;
			toggle_blinking_leds(state);
		}
		++blink_counter;
	}
	prev_state = state;
} */

/*void climby_set_leds(led_state_t state, int16_t bearing){
	clear_leds();
	switch(state){
	case CALIBRATION:
		set_rgb_led(LED2, -1, 0, 0); //Light up red
		set_rgb_led(LED4, -1, 0, 0);
		set_rgb_led(LED6, -1, 0, 0);
		set_rgb_led(LED8, -1, 0, 0);
		break;
	case MOVEMENT: //Movement state
		if(bearing <= 0)
			set_rgb_led(LED2, 0, 0, -1); //Light up blue
		if(bearing >= 0)
			set_rgb_led(LED8, 0, 0, -1);
		break;
	case TOP_REACHED: //Top reached state
		set_rgb_led(LED2, 0, -1, 0); //Light up green
		set_rgb_led(LED4, 0, -1, 0);
		set_rgb_led(LED6, 0, -1, 0);
		set_rgb_led(LED8, 0, -1, 0);
		break;
	}
}*/

/*void blink_leds(int state, int16_t bearing){
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
	else if(blink_counter >= blink_period_cycles){
		blink_counter = 0;
		blink_state = !blink_state;

		if(blink_state){
			leds_handler(state, bearing);
		}
		else
			clear_leds();
	}
	++blink_counter;
}
*/
