#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include "sensors/proximity.h"

static THD_WORKING_AREA(Proximity2_wa, 1024); // Why wa ? How many bits ?
static THD_FUNCTION(Proximity2, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    messagebus_t bus;
    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t prox_values;

    while(1){

    	time = chVTGetSystemTime();
    	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

 		if (SDU1.config->usbp->state != USB_ACTIVE) { // Skip printing if port not opened.
    		continue;
   		}

    	// Sensors info print: each line contains data related to a single sensor.
    	for (uint8_t i = 0; i < sizeof(prox_values.ambient)/sizeof(prox_values.ambient[0]); i++) {
    		//for (uint8_t i = 0; i < PROXIMITY_NB_CHANNELS; i++) {
    		chprintf((BaseSequentialStream *)&SDU1, "%4d,", prox_values.ambient[i]);
    		chprintf((BaseSequentialStream *)&SDU1, "%4d,", prox_values.reflected[i]);
    		chprintf((BaseSequentialStream *)&SDU1, "%4d", prox_values.delta[i]);
    		chprintf((BaseSequentialStream *)&SDU1, "\r\n");
    	}
    	chprintf((BaseSequentialStream *)&SDU1, "\r\n");

    	chThdSleepUntilWindowed(time, time + MS2ST(100)); // Refresh @ 10 Hz.

    	break;
    }
}

void proximity2_start(void){
	chThdCreateStatic(Proximity2_wa, sizeof(Proximity2_wa), NORMALPRIO, Proximity2, NULL);
}
