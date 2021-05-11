//#include "ch.h"
//#include "hal.h"
//#include <math.h>
//#include <usbcfg.h>
//#include <chprintf.h>
//#include <i2c_bus.h>

#include <leds.h>

#include <leds_handler.h>

void climby_toggle_leds(leds_state_t state, int16_t bearing){
	switch(state){
	case CALIBRATION:
		toggle_rgb_led(LED2, RED_LED, 255);
		toggle_rgb_led(LED4, RED_LED, 255);
		toggle_rgb_led(LED6, RED_LED, 255);
		toggle_rgb_led(LED8, RED_LED, 255);
		break;
	case MOVEMENT:
		if(bearing <= 0)
			set_rgb_led(LED2, 0, 0, 255); //set because in movement state LEDs do not blink
		if(bearing >= 0)
			set_rgb_led(LED8, 0, 0, 255);
		break;
	case TOP_REACHED:
		toggle_rgb_led(LED2, GREEN_LED, 255);
		toggle_rgb_led(LED4, GREEN_LED, 255);
		toggle_rgb_led(LED6, GREEN_LED, 255);
		toggle_rgb_led(LED8, GREEN_LED, 255);
	}
}

void climby_leds_handler(leds_state_t state, int16_t bearing){
	static leds_state_t prev_state = 0;
	static int8_t blink_counter = 0;
	if(prev_state == MOVEMENT){
		return;
	}
	if(state != prev_state){
		clear_leds();
		blink_counter = 0;
		prev_state = state;
		clear_leds();
		climby_toggle_leds(state, bearing);
	}
	else if(blink_counter >= BLINK_COUNTER_MAX){
		blink_counter = 0;
		climby_toggle_leds(state,bearing);
	}
	++blink_counter;
}

//OLDER FUNCTIONS I REMOVED :

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
