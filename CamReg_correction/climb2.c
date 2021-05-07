#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <i2c_bus.h>

//#include <main.h> //COPIED FROM TP5 ; I HAVE MY DOUBTS
#include <motors.h>
#include <sensors/imu.h>
#include <sensors/proximity.h>
#include <climb2.h>

static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//Convention used throughout: clockwise direction
// <=100 = left, 0 = straight, >=-100 = right

int16_t imu_bearing(int32_t acc_x, int32_t acc_y, int32_t acc_z){
	//Limit cases to avoid division by zero
	if(fabs(acc_x) <= IMU_RESOLUTION*IMU_EPSILON){
			return 0;
	}
	if(acc_y < IMU_TOP_MAX_Y && acc_z > IMU_TOP_MAX_Z){
		if(acc_x > 0)
			return 100;
		if(acc_x < 0)
			return -100;
	}
	return (int16_t)(atan2f(acc_x, acc_y)*200.0f/M_PI); //IS THIS CALCULATED IN FLOAT ?
}

int16_t prox_bearing(int prox_front_left, int prox_front_right, int prox_diag_left, int prox_diag_right){
	static bool direction_toggle = 0; //Arbitrary direction if all sensors are blocked
	static bool inc_toggle = 1; //Used in handling of direction toggle

	if(prox_front_left >= PROX_THRESHOLD && prox_front_right >= PROX_THRESHOLD){
		if(prox_diag_left >= PROX_THRESHOLD && prox_diag_right >= PROX_THRESHOLD){
			//In this case the robot makes a sharp turn in an arbitrary direction
			//SHARP turn : returns +-1 as opposed to <1
			//Direction is toggled to prevent back-and-forth loops improve climb efficiency (guess right turn as often as left)
			inc_toggle=1;
			return (1-2*direction_toggle)*100; //+100 or -100
		}

		if(inc_toggle){ //Increment the direction toggle if head-on collision avoided
			inc_toggle=0;
			++direction_toggle;
		}

		if(prox_diag_left >= PROX_THRESHOLD){
			return -prox_diag_left*100/(2*PROX_MAX); //Smaller angle correction for 45° sensors
		}
		if(prox_diag_right >= PROX_THRESHOLD){
			return prox_diag_right*100/(2*PROX_MAX);
		}
	}

	if(inc_toggle){ //Increment the direction toggle if head-on collision avoided
		inc_toggle=0;
		++direction_toggle;
	}

	if(prox_front_left >= PROX_THRESHOLD){ //If obstacle on left then turn right
		return prox_front_left*100/PROX_MAX;
	}
	if(prox_front_right >= PROX_THRESHOLD){ //If obstacle on right turn then left
		return -prox_front_right*100/PROX_MAX;
	}
	if(prox_diag_left >= PROX_THRESHOLD){
		return prox_diag_left*100/(2*PROX_MAX); //Smaller angle correction for 45° sensors
	}
	if(prox_diag_right >= PROX_THRESHOLD){
		return -prox_diag_right*100/(2*PROX_MAX);
	}
	return 0;
}

//Basic PID
void move (int16_t bearing){
	static int16_t bearing_prev = 0;
	static int16_t bearingI = 0;

	if(fabs(bearingI) < 1 || (bearingI >= 1 && bearing < 0) || (bearingI <= -1 && bearing > 0)){ //Prevent saturation
		bearingI += bearing ;
	}

	int16_t delta = Kp*bearing + Kd*(bearing - bearing_prev)+ Ki*bearingI;

	bearing_prev = bearing;
//	chprintf((BaseSequentialStream *)&SD3, "Delta = %d", delta);
//	chprintf((BaseSequentialStream *)&SD3, "Speed_L = %d", SPEED_BASE + SPEED_MAX_COEFF*MOTOR_SPEED_LIMIT*delta);
//	chprintf((BaseSequentialStream *)&SD3, "Speed_R = %d", SPEED_BASE - SPEED_MAX_COEFF*MOTOR_SPEED_LIMIT*delta);

	left_motor_set_speed(SPEED_BASE + SPEED_MAX_COEFF*MOTOR_SPEED_LIMIT*delta);
	right_motor_set_speed(SPEED_BASE - SPEED_MAX_COEFF*MOTOR_SPEED_LIMIT*delta);
}

static THD_WORKING_AREA(waSetPath, 512);

static THD_FUNCTION(SetPath, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    int16_t offset_x;
    int16_t offset_y;
    int16_t offset_z;

//Calibration of sensors
    do{//Protection in case of calibration with the robot tilted
    	calibrate_acc(); //Collects samples for calibration
    	offset_x = get_acc_offset(X_AXIS);
    	offset_y = get_acc_offset(Y_AXIS);
    	offset_z = get_acc_offset(Z_AXIS);
    	//chprintf((BaseSequentialStream *)&SD3, "Offset_Z = %d \r\n", offset_z);
    	chprintf((BaseSequentialStream *)&SD3, "CalibratingOff...");
    }while(offset_z < IMU_OFFSET_MAX || offset_z > IMU_OFFSET_MIN);//Calibrating

    //May not be necessary because the offset is variable and depends on the ambient light
    do{//Protection in case of calibration with the sensors covered
    	calibrate_ir();
    	chprintf((BaseSequentialStream *)&SD3, "CalibratingIR...");
    }while(get_prox(PROX_FRONT_LEFT)-get_calibrated_prox(PROX_FRONT_LEFT) >= PROX_OFFSET_MAX ||
    		get_prox(PROX_FRONT_RIGHT)-get_calibrated_prox(PROX_FRONT_RIGHT) >= PROX_OFFSET_MAX ||
			get_prox(PROX_DIAG_LEFT)-get_calibrated_prox(PROX_DIAG_LEFT) >= PROX_OFFSET_MAX ||
			get_prox(PROX_DIAG_RIGHT)-get_calibrated_prox(PROX_DIAG_RIGHT) >= PROX_OFFSET_MAX);

    systime_t time;

//Declaration of variables

    int16_t acc_x_buffer[IMU_BUFFER_SIZE_XY];
    int16_t acc_y_buffer[IMU_BUFFER_SIZE_XY];
    int16_t acc_z_buffer[IMU_BUFFER_SIZE_Z];

    int buffer_place_xy = 0;
    int buffer_place_z = 0;

    int32_t acc_x_averaged=0;
    int32_t acc_y_averaged=0;
    int32_t acc_z_averaged=0;

    int prox_front_left;
    int prox_front_right;
    int prox_diag_left;
    int prox_diag_right;
    int prox_left;
    int prox_right;

    int16_t bearing_prox;
    int16_t bearing_imu;
    int16_t bearing ;

    while(1){
    	time = chVTGetSystemTime();

//Collect and print all values
    	//It's redundant to assign memory space unless variables are called more than once
    	//Done here because all variables are also printed
    	//Remove for final version

    	//Calculated instantaneous acceleration values
    	acc_x_buffer[buffer_place_xy]= get_acc(X_AXIS)-offset_x ;
    	acc_y_buffer[buffer_place_xy]= get_acc(Y_AXIS)-offset_y ;
    	acc_z_buffer[buffer_place_z]= get_acc(Z_AXIS)-offset_z ;
    	//chprintf((BaseSequentialStream *)&SD3, "acc_y = %d", get_acc(Y_AXIS));
    	//chprintf((BaseSequentialStream *)&SD3, "offy = %d", offset_y);
    	//Increment position on the buffer
    	if(buffer_place_xy < IMU_BUFFER_SIZE_XY){
    		++buffer_place_xy;
    	}
    	else{
    		buffer_place_xy = 0;
    	}
    	if(buffer_place_z < IMU_BUFFER_SIZE_Z){
    	    ++buffer_place_z;
    	}
    	else{
    		buffer_place_z = 0;
    	}

    	//Calculate running average for acceleration values
    	for(int i=0; i<IMU_BUFFER_SIZE_XY; ++i){
    		acc_x_averaged += acc_x_buffer[i];
    		acc_y_averaged += acc_y_buffer[i];
    	}
    	for(int i=0; i<IMU_BUFFER_SIZE_Z; ++i){
    		acc_z_averaged += acc_z_buffer[i];
    	}


    	acc_x_averaged = acc_x_averaged / IMU_BUFFER_SIZE_XY;
    	acc_y_averaged = acc_y_averaged / IMU_BUFFER_SIZE_XY;
    	acc_z_averaged = acc_z_averaged / IMU_BUFFER_SIZE_Z;

    	//Print averaged accelerometer values:
    	//chprintf((BaseSequentialStream *)&SD3, "Acc_X = %d \r\n", acc_x_averaged);
    	//chprintf((BaseSequentialStream *)&SD3, "Acc_Y = %d \r\n", acc_y_averaged);
    	//chprintf((BaseSequentialStream *)&SD3, "Acc_Z = %d \r\n", acc_z_averaged);

    	//Collect the proximity values
    	prox_front_left = get_calibrated_prox(PROX_FRONT_LEFT);
    	prox_front_right = get_calibrated_prox(PROX_FRONT_RIGHT);
    	prox_diag_left = get_calibrated_prox(PROX_DIAG_LEFT);
    	prox_diag_right = get_calibrated_prox(PROX_DIAG_RIGHT);
    	prox_left = get_calibrated_prox(PROX_LEFT);
        prox_right = get_calibrated_prox(PROX_RIGHT);

        //Print proximity values
        chprintf((BaseSequentialStream *)&SD3, "Prox_FL = %d", prox_front_left);
        chprintf((BaseSequentialStream *)&SD3, "Prox_FR = %d", prox_front_right);
        chprintf((BaseSequentialStream *)&SD3, "Prox_DL = %d", prox_diag_left);
        chprintf((BaseSequentialStream *)&SD3, "Prox_DR = %d", prox_diag_right);
        chprintf((BaseSequentialStream *)&SD3, "Prox_L = %d", prox_left);
        chprintf((BaseSequentialStream *)&SD3, "Prox_R = %d", prox_right);

    	if((acc_x_averaged < IMU_TOP_MAX_X && acc_x_averaged > IMU_TOP_MIN_X) &&
    		(acc_y_averaged < IMU_TOP_MAX_Y && acc_y_averaged > IMU_TOP_MIN_Y) &&
    	    (acc_z_averaged < IMU_TOP_MAX_Z && acc_z_averaged > IMU_TOP_MIN_Z)){

    		left_motor_set_speed(0);
    		right_motor_set_speed(0);

    		chprintf((BaseSequentialStream *)&SD3, "TOP REACHED");

    		//chThdSleepUntilWindowed(time, time + MS2ST(10)); //Slows down the cycle by a factor of 10 -> idle mode
    	}

         else{
        	 bearing_prox = prox_bearing(prox_front_left, prox_front_right, prox_diag_left, prox_diag_right);
        	 //chprintf((BaseSequentialStream *)&SD3, "Bearing_PROX = %.4f \r\n", bearing_prox);
        	 bearing_imu = imu_bearing(acc_x_averaged, acc_y_averaged, acc_z_averaged);
        	 //chprintf((BaseSequentialStream *)&SD3, "Bearing_IMU = %.4f \r\n", bearing_imu);
        	 bearing = (1-bearing_prox/100)*bearing_imu + bearing_prox;
        	 //chprintf((BaseSequentialStream *)&SD3, "Bearing_RES = %.4f", bearing);
        	 move(bearing);
         }

    	chThdSleepUntilWindowed(time, time + MS2ST(10)); //100 Hz
    }
}

void set_path_start(void){
	chThdCreateStatic(waSetPath, sizeof(waSetPath), NORMALPRIO, SetPath, NULL);
}
