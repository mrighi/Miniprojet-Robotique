#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h> //COPIED FROM TP5 ; I HAVE MY DOUBTS
#include <motors.h>
#include <climb.h>

//I assumed the robot moves along its own y axis
//In fact y axis points backwards

double imu_angle(){ //That we have doubles is most certainly not good
	if(get_acc_filtered(Z_AXIS, IMU_SAMPLE_SIZE)-offset_z >= 1 - IMU_THRESHOLD){
		return 0;  //Stop the motors Maybe blink a led or smth ?
	}
	else if(get_acc_filtered(Y_AXIS, IMU_SAMPLE_SIZE)-offset_y <= IMU_THREASHOLD && get_acc_filtered(X_AXIS, IMU_SAMPLE_SIZE)-offset_x > 0){ //arctan gives pi/2 case
		return 1.57 ; //pi/2
	}
	else if(get_acc_filtered(Y_AXIS, IMU_SAMPLE_SIZE)-offset_y <= IMU_THRESHOLD && get_acc_filtered(X_AXIS, IMU_SAMPLE_SIZE)-offset_x < 0){
		return -1.57 ; //-pi/2
	}
	else{
		return atan((get_acc_filtered(X_AXIS, IMU_SAMPLE_SIZE)-offset_x)/(get_acc_filtered(Y_AXIS, IMU_SAMPLE_SIZE)-offset_y));
	}
}

int16_t prox_angle(){ //Very uncertain about the variable type
	//Left in the code just in case but this should be useless
	weight_left=1;
	weight_left=1;
	if(get_calibrated_prox(FRONT_RIGHT)<PROX_THRESHOLD){
		weight_left+=5 ;
	}
	else if(get_calibrated_prox(FRONT_LEFT)<PROX_THRESHOLD){
		weight_right+=5 ;
	}
	if(get_calibrated_prox(FRONT__DIAG_RIGHT)<PROX_THRESHOLD){ //not elseif because if the obstacle stretches further, turn more
		weight_left+=3 ;
	}
	else if(get_calibrated_prox(FRONT__DIAG_LEFT)<PROX_THRESHOLD){
		weight_right+=3 ;
	}
	return ??? //I forgot the syntax for this
}
//should give n different cases, depending on which sensors are tripped, and corresponding
//predetermined angles for each case

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
    	//Penalty optmisation problem
    	//angle_res=COEFF_IMU * imu_angle() + COEFF_PROX*prox_angle();

    	chThdSleepUntilWindowed(time, time + MS2ST(10)); //100 Hz
    }
}

void set_path_start(void){
	chThdCreateStatic(waSetPath, sizeof(waSetPath), NORMALPRIO, SetPath, NULL);
}
