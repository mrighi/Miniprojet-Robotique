#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <i2c_bus.h>

#include <main.h> //COPIED FROM TP5 ; I HAVE MY DOUBTS
#include <motors.h>
#include <sensors/imu.h>
#include <sensors/proximity.h>
#include <climb.h>

static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//I feel like it would be smarter to use a global or static variable
//That way this function runs only once and not continually
bool top_reached(int16_t offset_z){
	if(get_acceleration(Z_AXIS)-offset_z >= 1 - IMU_THRESHOLD){
		return 1; //Top has been reached
	}
	return 0; //Top has not been reached
}

//Using double because idk how to get arctan in float SHOULD BE CHANGED !!!
//Using degrees to be able to use ints
//Angles in trigonometric direction
double imu_bearing(int16_t offset_x, int16_t offset_y){
	if(get_acceleration(Y_AXIS)-offset_y <= IMU_THRESHOLD && get_acceleration(X_AXIS)-offset_x > 0){
		return 90 ;
	}
	else if(get_acceleration(Y_AXIS)-offset_y <= IMU_THRESHOLD && get_acceleration(X_AXIS)-offset_x < 0){
		return -90 ;
	}
	return atan((get_acceleration(X_AXIS)-offset_x)/(get_acceleration(Y_AXIS)-offset_y));
}

//Using double to match imu_bearing
//THIS IS A SIMPLIFIED VERSION FOR TESTING
double prox_bearing(void){
	/*bool coll__front_left = get_calibrated_prox(PROX_FRONT_LEFT);
	bool coll_diag_left = get_calibrated_prox(PROX_DIAG_LEFT);
	bool coll_front_right = get_calibrated_prox(PROX_FRONT_RIGHT);
	bool coll_diag_right = get_calibrated_prox(PROX_DIAG_RIGHT);
	bool coll_right = get_calibrated_prox(PROX_RIGHT);
	bool coll_left = get_calibrated_prox(PROX_LEFT);*/

	if(get_calibrated_prox(PROX_FRONT_LEFT) <= PROX_THRESHOLD){
		return -30 ;
	}
	if(get_calibrated_prox(PROX_FRONT_LEFT) <= PROX_THRESHOLD){
		return 30 ;
	}
	return 0;

	//Variant: use a static variable
	//Should prevent flip-flopping, but slower redressement
	/*
	static double bearing = 0;
	if(get_calibrated_prox(PROX_FRONT_LEFT) <= PROX_THRESHOLD and get_calibrated_prox(PROX_FRONT_LEFT) <= PROX_THRESHOLD){
		bearing += 90 //plus by convention
		return bearing ;
	}
	else if(get_calibrated_prox(PROX_FRONT_LEFT) <= PROX_THRESHOLD){
		bearing -= 30 ;
		return bearing ;
	}
	else if(get_calibrated_prox(PROX_FRONT_LEFT) <= PROX_THRESHOLD){
		bearing += 30;
		return bearing ;
	}
	else{
		bearing -= (fabs(bearing)/bearing) * 5 ;
		return bearing;
	}*/
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
    int angle_res;
    while(1){
    	time = chVTGetSystemTime();

    	if(top_reached(offset_z)){
    		left_motor_set_speed(0);
    		right_motor_set_speed(0);
    	}
    	else{ //VERY VERY BAD JUST FOR TESTING
    		angle_res=COEFF_IMU * imu_bearing(offset_x, offset_y) + COEFF_PROX*prox_bearing(); //Penalty optmisation problem
    		left_motor_set_speed(SPEED+angle_res);
    		right_motor_set_speed(SPEED-angle_res);
    	}

    	chThdSleepUntilWindowed(time, time + MS2ST(10)); //100 Hz
    }
}

void set_path_start(void){
	chThdCreateStatic(waSetPath, sizeof(waSetPath), NORMALPRIO, SetPath, NULL);
}
