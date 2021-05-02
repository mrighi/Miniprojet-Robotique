//MUST CORRECT:
//MAX_SPEED is already in motors.h
//Use filtered accelerometer values
//Change bearing to int8_t and on -100 to 100

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
#include <climb2.h>

static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//Convention used throughout:
// 1 = left, 0 = straight, -1 = right (trigonometric direction)

float imu_bearing(int16_t acc_x, int16_t acc_y){
	//Limit cases to avoid division by zero
	if(acc_y <= IMU_EPSILON*2/IMU_RESOLUTION && acc_x > 0){
		return 1 ;
	}
	if(acc_y <= IMU_EPSILON*2/IMU_RESOLUTION && acc_x < 0){
		return -1 ;
	}
	if(fabs(acc_x) <= IMU_EPSILON*2/IMU_RESOLUTION){
			return 0;
		}
	if((acc_x>0 && acc_y >0) || (acc_x<0 && acc_y <0) ){ //Use zeros because epsilon cases are already checked
		return 1;
	}
	if((acc_x<0 && acc_y >0) || (acc_x<0 && acc_y >0) ){
		return -1;
	}
}

float prox_bearing(int prox_front_left, int prox_front_right, int prox_diag_left, int prox_diag_right){
	static bool direction_toggle = 0; //Arbitrary direction if all sensors are blocked
	if(prox_front_left >= PROX_THRESHOLD && prox_front_right >= PROX_THRESHOLD){
		if(prox_diag_left >= PROX_THRESHOLD && prox_diag_right <= PROX_THRESHOLD){
			//In this case the robot makes a sharp turn in an arbitrary direction
			//SHARP turn : returns +-1 as opposed to <1
			//Direction is toggled every cycle, to prevent back-and-forth loops and improve efficiency
			++direction_toggle; //Assume spg at least one sensor is cleared after one cycle
			return 1-2*direction_toggle; //+1 or -1
		}
		if(prox_diag_left >= PROX_THRESHOLD){
			return -prox_diag_left/(2*PROX_MAX); //Smaller angle correction for 45° sensors
		}
		if(prox_diag_right >= PROX_THRESHOLD){
			return prox_diag_right/(2*PROX_MAX);
		}
	}
	if(prox_front_left >= PROX_THRESHOLD){
		return -prox_front_left/PROX_MAX;
	}
	if(prox_front_right >= PROX_THRESHOLD){
		return prox_front_right/PROX_MAX;
	}
	return 0;
}

//Control in speed, not position, to limit accelerations
//Smoother speeds profile = optimized climb
//SPEED_INC_COEFF controls the transient of the speed : 1 = no transient, ->0 = long transient
void move(float bearing){
	static float speed_left = 1;
	static float speed_right = 1;
	speed_left = speed_left + SPEED_INC_COEFF*bearing ;
	speed_right = speed_right - SPEED_INC_COEFF*bearing ;
	left_motor_set_speed(SPEED_MAX*speed_left);
	right_motor_set_speed(SPEED_MAX*speed_right);
}

static THD_WORKING_AREA(waSetPath, 256);

static THD_FUNCTION(SetPath, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    int16_t offset_x = 0;
    int16_t offset_y = 0;
    int16_t offset_z = 0;

    //Protection in case of calibration with the sensors covered
    do{
    	calibrate_acc(); //Collects samples for calibration
    	offset_x = get_acc_offset(X_AXIS);
    	offset_y = get_acc_offset(Y_AXIS);
    	offset_z = get_acc_offset(Z_AXIS);
    }while((offset_x >= PROX_OFFSET_MAX) || (offset_y >= PROX_OFFSET_MAX) || (offset_z >= PROX_OFFSET_MAX));

    systime_t time;

    int16_t acc_x_calibrated;
    int16_t acc_y_calibrated;
    int16_t acc_z_calibrated;

    int prox_front_left;
    int prox_front_right;
    int prox_diag_left;
    int prox_diag_right;

    float bearing ;

    while(1){
    	time = chVTGetSystemTime();

//Collect and print all values
    	//It's redundant to assign memory space unless variables are called more than once
    	//Done here because all variables are also printed
    	//Remove for final version

    	//Collect acceleration values
    	acc_x_calibrated=(get_acceleration(X_AXIS)-offset_x);
    	acc_y_calibrated=(get_acceleration(Y_AXIS)-offset_y);
    	acc_z_calibrated=(get_acceleration(Z_AXIS)-offset_z);

    	//Print the offsets:
    	chprintf((BaseSequentialStream *)&SD3, "Offset_X = %d \r\n", offset_x);
    	chprintf((BaseSequentialStream *)&SD3, "Offset_Y = %d \r\n", offset_y);
    	chprintf((BaseSequentialStream *)&SD3, "Offset_Z = %d \r\n", offset_z);

    	//Print the accelerometer values:
    	chprintf((BaseSequentialStream *)&SD3, "Acc_X = %d \r\n", acc_x_calibrated);
    	chprintf((BaseSequentialStream *)&SD3, "Acc_Y = %d \r\n", acc_y_calibrated);
    	chprintf((BaseSequentialStream *)&SD3, "Acc_Z = %d \r\n", acc_z_calibrated);

    	//Collect the proximity values
    	prox_front_left = get_calibrated_prox(PROX_FRONT_LEFT);
    	prox_front_right = get_calibrated_prox(PROX_FRONT_RIGHT);
    	prox_diag_left = get_calibrated_prox(PROX_DIAG_LEFT);
    	prox_diag_right = get_calibrated_prox(PROX_DIAG_RIGHT);
    	//prox_left = get_calibrated_prox(PROX_LEFT);
        //prox_right = get_calibrated_prox(PROX_RIGHT);

        //Print proximity values
        chprintf((BaseSequentialStream *)&SD3, "Prox_FL = %d \r\n", prox_front_left);
        chprintf((BaseSequentialStream *)&SD3, "Prox_FR = %d \r\n", prox_front_right);
        chprintf((BaseSequentialStream *)&SD3, "Prox_DL = %d \r\n", prox_diag_left);
        chprintf((BaseSequentialStream *)&SD3, "Prox_DR = %d \r\n", prox_diag_right);
        //chprintf((BaseSequentialStream *)&SD3, "Prox_L = %d \r\n", prox_left);
        //chprintf((BaseSequentialStream *)&SD3, "Prox_R = %d \r\n", prox_right);

//Set motor speeds accordingly
        //Case top reached
         if(acc_z_calibrated <= -(1 - IMU_EPSILON)*2/IMU_RESOLUTION){ //Minus because z axis points up // 2/res corresponds to g
        	 left_motor_set_speed(0);
        	 right_motor_set_speed(0);
        	 chprintf((BaseSequentialStream *)&SD3, "TOP REACHED");
         }
         else{
        	 bearing = COEFF_PROX*imu_bearing(acc_x_calibrated, acc_y_calibrated) + COEFF_IMU*prox_bearing(prox_front_left, prox_front_right, prox_diag_left, prox_diag_right);
        	 chprintf((BaseSequentialStream *)&SD3, "Bearing = %.4f", bearing);
        	 move(bearing);
         }

    	chThdSleepUntilWindowed(time, time + MS2ST(10)); //100 Hz
    }
}

void set_path_start(void){
	chThdCreateStatic(waSetPath, sizeof(waSetPath), NORMALPRIO, SetPath, NULL);
}
