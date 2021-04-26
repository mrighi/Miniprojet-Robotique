/*We would use a single thread to:
 *	- Read IMU values
 *	- Read prox values
 *	- Calculate a path
 *	- Set the motors to that path
 */

#include "ch.h"
#include "hal.h"
#include <math.h> //May or may not be necessary
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h> //COPIED FROM TP5 ; I HAVE MY DOUBTS
#include <motors.h>
#include <climb.h>

int16_t imu_angle(){ //Very uncertain about the variable type

}

int16_t prox_angle(){
	if(get_acc_filtered(Z_AXIS)-offset_z >= 1 - PLATEAU_DETECTION_THRESHOLD){
		return 0;  //Maybe blink a led or smth ?
	}
	else{

	}
}

static THD_WORKING_AREA(waSetPath, 256); //Should not need much memory (no tables)

static THD_FUNCTION(SetPath, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    calibrate_acc(); //Collects samples for calibration
    //Offsets calculated in advance to speed up program
    int16_t offset_x = get_acc_offset(X_AXIS);
    int16_t offset_y = get_acc_offset(Y_AXIS);
    int16_t offset_z = get_acc_offset(Z_AXIS);

    // Functions to use in the thread:
    //get_calibrated_prox( /*0 to 7*/ )

    //get_acc_offset( /* 0 to 2 = x,y,z */ );

    //get_acc_filtered( /* 0 to 2*/ , /* filter size = nb samples*/ );

    //left_motor_set_speed( /* int < 1100 */ );
    //right_motor_set_speed();

    systime_t time;
    while(1){
    	time = chVTGetSystemTime();

    	//Speed is a weighted average of prox and ir values

    	chThdSleepUntilWindowed(time, time + MS2ST(10)); //100 Hz
    }
}

void set_path_start(void){
	chThdCreateStatic(waSetPath, sizeof(waSetPath), NORMALPRIO, SetPath, NULL);
}
