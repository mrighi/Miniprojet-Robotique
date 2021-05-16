#ifndef LEDS_HANDLER_H_
#define LEDS_HANDLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#define INTENSITY_MAX			255		//RGB LEDS
#define INTENSITY_MED			150

#define CALIBRATION_COUNTER_MAX	1
#define TOPREACHED_COUNTER_MAX	5

//States of the robot where the behavior of the leds has to be different
typedef enum {
	CALIBRATION,
	MOVEMENT,
	TOP_REACHED,
}leds_state_t;

/**
* @brief	Toggle the four rgb leds to orange
* 			At the first call lets are set, at the next they are cleared, etc.
*/
void toggle_calibration_leds(void);

/**
* @brief	Light the left and/or right front rgb leds blue, depending on the sign of bearing
*
* @param	Bearing value (can be any int)
*/
void set_movement_leds(int8_t rotation);

/**
* @brief	Toggle the four rgb leds to green
* 			At the first call lets are set, at the next they are cleared, etc.
*
*/
void toggle_topreached_leds(void);

/**
* @brief	Used to handle leds behaviour:
* 			In CALIBRATION and TOP_REACHED state blinks the leds, in MOVEMENT activates the corresponding led(s)
*
* @param	Movement state : CALIBRATION, MOVEMENT, TOP_REACHED
* 			Bearing to determine which led to turn on in movement state
*/
void climby_leds_handler(leds_state_t state, int8_t rotation);

#endif /* LEDS_HANDLER_H_ */
