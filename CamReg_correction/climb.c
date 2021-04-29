/*
 * QUESTIONS:
 *  - Use get_acc_filtered ?
 *  - Am I using offsets correctly ?
 *  - Angles are in float, can we get in int ?
 */
//MUST CORRECT:
//PROBABLY SWITCH ACC_X AND ACC_X_CALIBRATED
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

//Using double because idk how to get arctan in float SHOULD BE CHANGED !!!
//Using degrees to be able to use ints
//Angles are in trigonometric direction
float imu_bearing(int16_t acc_x_calibrated, int16_t acc_y_calibrated){
	if(acc_y_calibrated <= IMU_EPSILON*g && acc_y_calibrated > 0){
		return 90 ;
	}
	else if(acc_y_calibrated <= IMU_EPSILON*g && acc_x_calibrated < 0){
		return -90 ;
	}
	return 57.3f*atan2(acc_x_calibrated,acc_y_calibrated);
	//57.3 is to convert to degree
	//f so it's compiled as a float
}

//Using double to match imu_bearing
//THIS IS A SIMPLIFIED VERSION FOR TESTING
float prox_bearing(void){
	/*bool coll__front_left = get_calibrated_prox(PROX_FRONT_LEFT);
	bool coll_diag_left = get_calibrated_prox(PROX_DIAG_LEFT);
	bool coll_front_right = get_calibrated_prox(PROX_FRONT_RIGHT);
	bool coll_diag_right = get_calibrated_prox(PROX_DIAG_RIGHT);
	bool coll_right = get_calibrated_prox(PROX_RIGHT);
	bool coll_left = get_calibrated_prox(PROX_LEFT);*/

	//Redundant to assign memory space unless these are used more than once
	//Done here to print
	int prox_left= get_calibrated_prox(PROX_FRONT_LEFT);
	int prox_right= get_calibrated_prox(PROX_FRONT_RIGHT);

	//Print proximity values
	//Does .2f work on int ?
	chprintf((BaseSequentialStream *)&SD3, "Prox_L = %i \r\n", prox_left);
	chprintf((BaseSequentialStream *)&SD3, "Prox_R = %i \r\n", prox_right);

	if(prox_left >= PROX_THRESHOLD){
		return -30 ; //In trigonometric direction, corresponds to turning clockwise
	}
	if(prox_right >= PROX_THRESHOLD){
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
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    float angle_imu;
    float angle_prox;
    float angle_res;
    while(1){
    	time = chVTGetSystemTime();

    	//It's redundant to assign memory space unless these are called more than once
    	//Done here so it can be printed
    	acc_x=(get_acceleration(X_AXIS)-offset_x)*(2*g/IMU_RESOLUTION);
    	acc_y=(get_acceleration(Y_AXIS)-offset_y)*(2*g/IMU_RESOLUTION);
    	acc_z=(get_acceleration(Z_AXIS)-offset_z)*(2*g/IMU_RESOLUTION);

    	//Print the offsets:
    	 chprintf((BaseSequentialStream *)&SD3, "Offset_X = %i \r\n", offset_x);
    	 chprintf((BaseSequentialStream *)&SD3, "Offset_Y = %i \r\n", offset_y);
    	 chprintf((BaseSequentialStream *)&SD3, "Offset_Z = %i \r\n", offset_z);

    	 //Print the accelerometer values:
    	 chprintf((BaseSequentialStream *)&SD3, "Acc_X = %i \r\n", acc_x);
    	 chprintf((BaseSequentialStream *)&SD3, "Acc_Y = %i \r\n", acc_y);
    	 chprintf((BaseSequentialStream *)&SD3, "Acc_Z = %i \r\n", acc_z);

    	/*if(acc_z <= -(1 - IMU_EPSILON)*g){ //Top reached //Minus because z axis points up
    		left_motor_set_speed(0);
    		right_motor_set_speed(0);
    		//MAY WELL BE A SOURCE OF ERRORS
    		chprintf((BaseSequentialStream *)&SD3, "TOP REACHED");
    	}*/
    	//else{ //VERY VERY BAD JUST FOR TESTING
    		angle_imu = imu_bearing(acc_x, acc_y);
    		angle_prox = prox_bearing();
    		angle_res=COEFF_IMU * imu_bearing(acc_x, acc_y) + COEFF_PROX*prox_bearing(); //Penalty optmization problem

    		//Print angles values
    		chprintf((BaseSequentialStream *)&SD3, "Angle_IMU = %.2f \r\n", angle_imu);
    		chprintf((BaseSequentialStream *)&SD3, "Angle_PROX = %.2f \r\n", angle_prox);
    		chprintf((BaseSequentialStream *)&SD3, "Angle_RES = %.2f \r\n", angle_res);

    		left_motor_set_speed(SPEED-angle_res);
    		right_motor_set_speed(SPEED+angle_res);
    	//}

    	chThdSleepUntilWindowed(time, time + MS2ST(10)); //100 Hz
    }
}

void set_path_start(void){
	chThdCreateStatic(waSetPath, sizeof(waSetPath), NORMALPRIO, SetPath, NULL);
}
