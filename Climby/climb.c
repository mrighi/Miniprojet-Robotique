#include "ch.h"
#include "hal.h"
#include <math.h>

#include <motors.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <IMU_handler.h>
#include <leds_handler.h>
#include <climb.h>

static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

int8_t imu_bearing(int16_t acc_x, int16_t acc_y){
	if(fabs(acc_x) <= IMU_GO_STRAIGHT_THRESHOLD && acc_y > 0)
			return 0;
	//Limit case to avoid division by zero + if the robot is more than 90 degrees off
	if(acc_y < IMU_TOP_MAX_Y && acc_x >= 0)
			return -BEARING_MAX;
	if(acc_y < IMU_TOP_MAX_Y && acc_x < 0)
			return BEARING_MAX;
	//Calculated in float [-1,1] then converted to int [-100,100] to optimize processing time
	return (int8_t)(-atan2f(acc_x, acc_y)*ATAN_TO_BEARING);
}

int8_t prox_bearing(uint16_t dist_mm){
	static int8_t bearing_prox = 0; //Static because decremented once the obstacle is no longer in line of sight
	static bool direction = 0; //Direction of the rotation : 0=left, 1=right

	if(dist_mm <= PROX_DIST_MIN) //Obstacle detected
		bearing_prox = (1-2*direction)*PROX_CORRECTION;
	else if(bearing_prox != 0){
		if(fabs(bearing_prox) - PROX_DEC_COEFF > 0) //Next decrement does not change sign
			bearing_prox = (1-2*direction)*(fabs(bearing_prox)-PROX_DEC_COEFF); //Decrement bearing_prox
		else{
			bearing_prox = 0;
			direction = !direction;
		}
	}
	return bearing_prox;
}

void move (int8_t bearing){
	static int16_t bearingI = 0;
	if(fabs(bearingI) < BEARING_I_MAX ||
			(bearingI >= BEARING_I_MAX && bearing < 0) ||
			(bearingI <= -BEARING_I_MAX && bearing > 0)) //Prevent saturation of I term
		bearingI += bearing ;

	int16_t delta_speed = Kp*bearing + (Kp*bearingI)/Ti;

	left_motor_set_speed(SPEED_BASE - delta_speed);
	right_motor_set_speed(SPEED_BASE + delta_speed);

	climby_leds_handler(MOVEMENT, bearing);
	//Led handling done here because real correction value delta_speed does not necessarily correspond to bearing
}

static THD_WORKING_AREA(waSetPath, 256);

static THD_FUNCTION(SetPath, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(climby_calibrate_acc()){
    	climby_leds_handler(CALIBRATION,0);
    }

    systime_t time;
    uint16_t ToF_dist_mm;
    int16_t acc[3] = {0};
    int8_t bearing_prox;
    int8_t bearing_imu;
    int8_t bearing_res;

    while(1){
    	time = chVTGetSystemTime();

        ToF_dist_mm = VL53L0X_get_dist_mm();

        get_averaged_acc(acc);

    	if((acc[X_AXIS] < IMU_TOP_MAX_X && acc[X_AXIS] > IMU_TOP_MIN_X) &&
    		(acc[Y_AXIS] < IMU_TOP_MAX_Y && acc[Y_AXIS] > IMU_TOP_MIN_Y) &&
    	    (acc[Z_AXIS] < IMU_TOP_MAX_Z && acc[Z_AXIS] > IMU_TOP_MIN_Z)){ //Top reached
    		left_motor_set_speed(0);
    		right_motor_set_speed(0);
    		climby_leds_handler(TOP_REACHED,0);
    	}
         else{
        	 bearing_prox = prox_bearing(ToF_dist_mm);
        	 bearing_imu = imu_bearing(acc[X_AXIS], acc[Y_AXIS]);
        	 bearing_res = ((PROX_CORRECTION-fabs(bearing_prox))*bearing_imu)/PROX_CORRECTION + bearing_prox; //Dynamic weighting
        	 move(bearing_res);
         }

    	chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void set_path_start(void){
	chThdCreateStatic(waSetPath, sizeof(waSetPath), NORMALPRIO, SetPath, NULL);
}
