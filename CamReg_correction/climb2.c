//MUST CORRECT:
//Change bearing to int8_t and on -100 to 100
//Put back the arctan in the imu bearing

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
	return atan2(acc_x, acc_y)*0.637f; //0.637 = 2/pi
}

float prox_bearing(int prox_front_left, int prox_front_right, int prox_diag_left, int prox_diag_right){
	static bool direction_toggle = 0; //Arbitrary direction if all sensors are blocked
	static bool inc_toggle = 1; //Used in handling of direction toggle

	if(prox_front_left >= PROX_THRESHOLD && prox_front_right >= PROX_THRESHOLD){
		if(prox_diag_left >= PROX_THRESHOLD && prox_diag_right >= PROX_THRESHOLD){
			//In this case the robot makes a sharp turn in an arbitrary direction
			//SHARP turn : returns +-1 as opposed to <1
			//Direction is toggled to prevent back-and-forth loops improve climb efficiency (guess right turn as often as left)
			inc_toggle=1;
			return 1-2*direction_toggle; //+1 or -1
		}

		if(inc_toggle){ //Increment the direction toggle if head-on collision avoided
			inc_toggle=0;
			++direction_toggle;
		}

		if(prox_diag_left >= PROX_THRESHOLD){
			return -prox_diag_left/(2*PROX_MAX); //Smaller angle correction for 45° sensors
		}
		if(prox_diag_right >= PROX_THRESHOLD){
			return prox_diag_right/(2*PROX_MAX);
		}
	}

	if(inc_toggle){ //Increment the direction toggle if head-on collision avoided
		inc_toggle=0;
		++direction_toggle;
	}

	if(prox_front_left >= PROX_THRESHOLD){ //If obstacle on left then turn right
		return -prox_front_left/PROX_MAX;
	}
	if(prox_front_right >= PROX_THRESHOLD){ //If obstacle on right turn then left
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

	//if prevents saturation of speed_x variables
	if(fabs(speed_left) < 1 || (speed_left >= 1 && bearing < 0) || (speed_left <=1 && bearing < 0)){
		speed_left += SPEED_INC_COEFF*bearing ;
	}
	if(fabs(speed_right) < 1 || (speed_right >= 1 && bearing > 0) || (speed_right <=1 && bearing < 0)){
		speed_right -= SPEED_INC_COEFF*bearing ;
	}

	chprintf((BaseSequentialStream *)&SD3, "Speed_LEFT = %.4f", speed_left);
	chprintf((BaseSequentialStream *)&SD3, "Speed_RIGHT = %.4f", speed_right);

	left_motor_set_speed(SPEED_MAX_COEFF*MOTOR_SPEED_LIMIT*speed_left);
	right_motor_set_speed(SPEED_MAX_COEFF*MOTOR_SPEED_LIMIT*speed_right);
}

static THD_WORKING_AREA(waSetPath, 256);

static THD_FUNCTION(SetPath, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    int16_t offset_x;
    int16_t offset_y;
    int16_t offset_z;

//Calibration of sensors
    //do{//Protection in case of calibration with the robot tilted
    	calibrate_acc(); //Collects samples for calibration
    	offset_x = get_acc_offset(X_AXIS);
    	offset_y = get_acc_offset(Y_AXIS);
    	offset_z = get_acc_offset(Z_AXIS);
    //}while(fabs(offset_z) <= (1/2 + 5*IMU_EPSILON)*IMU_RESOLUTION/2 || fabs(offset_z) <= (1/2 - 5*IMU_EPSILON)*IMU_RESOLUTION/2);

    //May not be necessary because the offset is variable and depends on the ambient light
    //do{//Protection in case of calibration with the sensors covered
    	calibrate_ir();
    //}while(get_prox(PROX_FRONT_LEFT)-get_calibrated_prox(PROX_FRONT_LEFT) >= PROX_OFFSET_MAX ||
    		//get_prox(PROX_FRONT_RIGHT)-get_calibrated_prox(PROX_FRONT_RIGHT) >= PROX_OFFSET_MAX ||
			//get_prox(PROX_DIAG_LEFT)-get_calibrated_prox(PROX_DIAG_LEFT) >= PROX_OFFSET_MAX ||
			//get_prox(PROX_DIAG_RIGHT)-get_calibrated_prox(PROX_DIAG_RIGHT) >= PROX_OFFSET_MAX);

    systime_t time;

//Declaration of variables
    int16_t acc_x_calibrated;
    int16_t acc_y_calibrated;
    int16_t acc_z_calibrated;

    int prox_front_left;
    int prox_front_right;
    int prox_diag_left;
    int prox_diag_right;

    float bearing_prox;
    float bearing_imu;
    float bearing ;

    while(1){
    	time = chVTGetSystemTime();

//Collect and print all values
    	//It's redundant to assign memory space unless variables are called more than once
    	//Done here because all variables are also printed
    	//Remove for final version

    	//Collect acceleration values
    	//Using non-filtered to maintain high frequency
    	acc_x_calibrated=(get_acc(X_AXIS)-offset_x);
    	acc_y_calibrated=(get_acc(Y_AXIS)-offset_y);
    	acc_z_calibrated=(get_acc(Z_AXIS)-offset_z);

    	//Print the offsets:
    	//chprintf((BaseSequentialStream *)&SD3, "Offset_X = %d \r\n", offset_x);
    	//chprintf((BaseSequentialStream *)&SD3, "Offset_Y = %d \r\n", offset_y);
    	//chprintf((BaseSequentialStream *)&SD3, "Offset_Z = %d \r\n", offset_z);

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
        //chprintf((BaseSequentialStream *)&SD3, "Prox_DL = %d \r\n", prox_diag_left);
        //chprintf((BaseSequentialStream *)&SD3, "Prox_DR = %d \r\n", prox_diag_right);
        //chprintf((BaseSequentialStream *)&SD3, "Prox_L = %d \r\n", prox_left);
        //chprintf((BaseSequentialStream *)&SD3, "Prox_R = %d \r\n", prox_right);

//Set motor speeds accordingly
        //Case top reached
        //Fix the condition to have a more acceptable threshold
         if(fabs(acc_z_calibrated) <= IMU_EPSILON*IMU_MAX/2){ //Minus because z axis points up
        	 left_motor_set_speed(0);
        	 right_motor_set_speed(0);
        	 chprintf((BaseSequentialStream *)&SD3, "TOP REACHED");
         }
         else{
        	 bearing_prox = prox_bearing(prox_front_left, prox_front_right, prox_diag_left, prox_diag_right);
        	 chprintf((BaseSequentialStream *)&SD3, "Bearing_PROX = %.4f \r\n", bearing_prox);
        	 bearing_imu = imu_bearing(acc_x_calibrated, acc_y_calibrated);
        	 chprintf((BaseSequentialStream *)&SD3, "Bearing_IMU = %.4f \r\n", bearing_imu);
        	 bearing = (1-bearing_prox)*bearing_imu + bearing_prox;
        	 chprintf((BaseSequentialStream *)&SD3, "Bearing_RES = %.4f", bearing);
        	 move(bearing);
         }

    	chThdSleepUntilWindowed(time, time + MS2ST(10)); //100 Hz
    }
}

void set_path_start(void){
	chThdCreateStatic(waSetPath, sizeof(waSetPath), NORMALPRIO, SetPath, NULL);
}
