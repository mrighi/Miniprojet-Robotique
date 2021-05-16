#include "ch.h"
#include "hal.h"

#include <leds.h>
#include <leds_handler.h>

void toggle_calibration_leds(void){
	static bool leds_state = 1; //Leds are turned on on first cycle
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
	static int counter = TOPREACHED_COUNTER_MAX; //Leds are turned on on first cycle
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
