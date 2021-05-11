#ifndef LEDS_HANDLER_H_
#define LEDS_HANDLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#define BLINK_COUNTER_MAX		5

//States of the robot where the behavior of the leds has to be different
typedef enum {
	CALIBRATION,
	MOVEMENT,
	TOP_REACHED,
}leds_state_t;

/**
* @brief	Toggle the four rgb leds
* 			In CALIBRATION state toggles the four leds red
* 			In TOP_REACHED state toggles the four leds green
*
* @param	Movement state : CALIBRATION, MOVEMENT, TOP_REACHED
*/
void toggle_blinking_leds(leds_state_t state);

/**
* @brief	Light the left and/or right front rgb leds blue, depending on the sign of bearing
*
* @param	Movement state : CALIBRATION, MOVEMENT, TOP_REACHED
* 			Bearing
*/
void set_movement_leds(int16_t bearing);

/**
* @brief	Used to handle leds behaviour:
* 			In CALIBRATION and TOP_REACHED state blinks the leds, in MOVEMENT activates the corresponding led(s)
*
* @param	Movement state : CALIBRATION, MOVEMENT, TOP_REACHED
* 			Bearing to determine which led to turn on in movement state
*/
void climby_leds_handler(leds_state_t state, int16_t bearing);

#endif /* LEDS_HANDLER_H_ */
