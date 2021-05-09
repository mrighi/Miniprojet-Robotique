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
#include <sensors/VL53L0X/VL53L0X.h> //ToF
#include <leds_handler.h>

#include <climb2.h>

static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//Convention used throughout: trigonometric direction
// (0,100] = right, 0 = straight, [-100, 0) = left

int16_t imu_bearing(int32_t acc_x, int32_t acc_y, int32_t acc_z){
	//Limit cases to avoid division by zero
	if(fabs(acc_x) <= IMU_RESOLUTION*IMU_EPSILON){
			return 0;
	}
	if(acc_y < IMU_TOP_MAX_Y && acc_z > IMU_TOP_MAX_Z){
		if(acc_x > 0)
			return -100;
		if(acc_x < 0)
			return 100;
	}
	//Analog correction value reduces overshoot
	return (int16_t)(-atan2f(acc_x, acc_y)*200.0f/M_PI); //IS THIS CALCULATED IN FLOAT ?
}

int16_t prox_bearing(uint16_t dist_mm){
	static bool direction = 0; //0 = left, 1 = right
	static bool switch_direction_flag = 0;
	static bool clear = 1; //0 = obstacle present

	static int16_t bearing_prox = 0;

	if(dist_mm <= 100){ //Obstacle detected
		if(!switch_direction_flag){
			switch_direction_flag = 1; //Once obstacle is avoided, direction variable is switched
			clear = 0;
		}
		bearing_prox = (1-2*direction)*100; //Constant correction
		//bearing_prox = (1-2*direction)*100*(1-dist_mm/100); //Linear correction normalized to +-100
		return bearing_prox;
	}

	if(clear){ //Reduce operations when clear
		return 0;
	}

	else{
		if(switch_direction_flag){ //Once obstacle is avoided, direction variable is switched
			direction = !direction;
			switch_direction_flag = 0;
		}

		bearing_prox = (1-2*direction)*(fabs(bearing_prox)-1); //Decrement bearing_prox

		if(fabs(bearing_prox) <= PROX_DEC_COEFF){
			clear = 1;
			bearing_prox = 0;
		}
		return bearing_prox;
	}
}

//PID regulator
void move (int16_t bearing){
	static int16_t bearing_prev = 0;
	static int16_t bearingI = 0;

	if(fabs(bearingI) < 1 || (bearingI >= 1 && bearing < 0) || (bearingI <= -1 && bearing > 0)){ //Prevent saturation
		bearingI += bearing ;
	}

	int16_t delta = Kp*bearing + Kd*(bearing - bearing_prev)+ Ki*bearingI;

	bearing_prev = bearing;

	//Test how far this is from [-100, 100]
	chprintf((BaseSequentialStream *)&SD3, "delta = %d ", delta);

	left_motor_set_speed(SPEED_BASE - SPEED_MAX_COEFF*MOTOR_SPEED_LIMIT*delta);
	right_motor_set_speed(SPEED_BASE + SPEED_MAX_COEFF*MOTOR_SPEED_LIMIT*delta);

	blink_leds(1, delta, LEDS_BLINK_PERIOD);
	//Led handling done here because real delta correction value does not necessarily correspond to bearing
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
    	blink_leds(0, 0, LEDS_BLINK_PERIOD);

    	calibrate_acc(); //Collects samples for calibration

    	offset_x = get_acc_offset(X_AXIS);
    	offset_y = get_acc_offset(Y_AXIS);
    	offset_z = get_acc_offset(Z_AXIS);
    }while(offset_z < IMU_OFFSET_MAX || offset_z > IMU_OFFSET_MIN);//Calibrating

    systime_t time;

//Declaration of variables

    int16_t acc_x_buffer[IMU_BUFFER_SIZE_XY] = {0};
    int16_t acc_y_buffer[IMU_BUFFER_SIZE_XY] = {0};
    int16_t acc_z_buffer[IMU_BUFFER_SIZE_Z] = {0};

    int buffer_place_xy = 0;
    int buffer_place_z = 0;

    //Separate variable for sum allows acc_..._averaged to be on 16 bits
    int32_t acc_x_sum=0;
    int32_t acc_y_sum=0;
    int32_t acc_z_sum=0;

    int16_t acc_x_averaged=0;
    int16_t acc_y_averaged=0;
    int16_t acc_z_averaged=0;

    uint16_t ToF_dist_mm;

    int16_t bearing_prox;
    int16_t bearing_imu;
    int16_t bearing ;

    while(1){
    	time = chVTGetSystemTime();

//Collect and print all values
    	//It's redundant to assign memory space unless variables are called more than once
    	//Done here because all variables are also printed
    	//Remove for final version

    	//Collect acceleration values
    	acc_x_sum -= acc_x_buffer[buffer_place_xy];
    	acc_y_sum -= acc_y_buffer[buffer_place_xy];
    	acc_z_sum -= acc_z_buffer[buffer_place_z];

    	acc_x_buffer[buffer_place_xy]= get_acc(X_AXIS)-offset_x ;
    	acc_y_buffer[buffer_place_xy]= get_acc(Y_AXIS)-offset_y ;
    	acc_z_buffer[buffer_place_z]= get_acc(Z_AXIS)-offset_z ;

    	acc_x_sum += acc_x_buffer[buffer_place_xy];
    	acc_y_sum += acc_y_buffer[buffer_place_xy];
    	acc_z_sum += acc_z_buffer[buffer_place_z];

    	acc_x_averaged = acc_x_sum / IMU_BUFFER_SIZE_XY;
    	acc_y_averaged = acc_y_sum / IMU_BUFFER_SIZE_XY;
    	acc_z_averaged = acc_z_sum / IMU_BUFFER_SIZE_Z;

    	//Print averaged accelerometer values:
    	chprintf((BaseSequentialStream *)&SD3, "Acc_X = %d \r\n", acc_x_averaged);
    	chprintf((BaseSequentialStream *)&SD3, "Acc_Y = %d \r\n", acc_y_averaged);
    	chprintf((BaseSequentialStream *)&SD3, "Acc_Z = %d \r\n", acc_z_averaged);

    	//Increment position on the buffers
    	buffer_place_xy = (buffer_place_xy + 1) % IMU_BUFFER_SIZE_XY ;
    	buffer_place_z = (buffer_place_z + 1) % IMU_BUFFER_SIZE_Z ;

        ToF_dist_mm = VL53L0X_get_dist_mm();

//Movement handling
    	if((acc_x_averaged < IMU_TOP_MAX_X && acc_x_averaged > IMU_TOP_MIN_X) &&
    		(acc_y_averaged < IMU_TOP_MAX_Y && acc_y_averaged > IMU_TOP_MIN_Y) &&
    	    (acc_z_averaged < IMU_TOP_MAX_Z && acc_z_averaged > IMU_TOP_MIN_Z)){ //Top reached

    		left_motor_set_speed(0);
    		right_motor_set_speed(0);

    		blink_leds(2, 0, LEDS_BLINK_PERIOD);

    		chprintf((BaseSequentialStream *)&SD3, "TOP REACHED");

    		//chThdSleepUntilWindowed(time, time + MS2ST(10)); //Slows down the cycle by a factor of 10 -> idle mode
    	}

         else{
        	 bearing_prox = prox_bearing(ToF_dist_mm);
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
