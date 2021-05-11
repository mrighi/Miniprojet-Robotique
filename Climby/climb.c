#include "ch.h"
#include "hal.h"
#include <math.h>
//#include <usbcfg.h>
//#include <i2c_bus.h>

//#include <main.h>
//#include <sensors/imu.h>
//#include <sensors/proximity.h>
//#include <leds.h>
#include <motors.h>
#include <sensors/VL53L0X/VL53L0X.h> //ToF

#include <IMU_handler.h>
#include <leds_handler.h>

#include <climb.h>

//REMOVE THIS IN FINAL VERSION!!!
#include <chprintf.h>

static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

int16_t imu_bearing(int32_t acc_x, int32_t acc_y, int32_t acc_z){
	if(fabs(acc_x) <= IMU_RESOLUTION*IMU_EPSILON) //Threshold case
			return 0;
	if(acc_y < IMU_TOP_MAX_Y && acc_z > IMU_TOP_MAX_Z){ //Limit case to avoid division by zero
		if(acc_x > 0)
			return -100;
		if(acc_x < 0)
			return 100;
	}
	//Calculated in float [-1,1] then converted to int [-100,100] to optimize processing time
	return (int16_t)(-atan2f(acc_x, acc_y)*200.0f/M_PI);
}

int16_t prox_bearing(uint16_t dist_mm){
	static bool direction = 0; //Direction of the robot : 0 = left, 1 = right

	static bool switch_direction_flag = 0; //Used to toggle direction
	static bool obstacle_cleared = 1; //Used to toggle direction : 0 = obstacle, 1 = obstacle cleared

	static int16_t bearing_prox = 0; //Static because decremented once the obstacle is no longer in line of sight

	if(dist_mm <= PROX_DIST_MIN){ //Obstacle detected
		if(!switch_direction_flag){
			switch_direction_flag = 1;
			obstacle_cleared = 0;
		}
		//bearing_prox = (1-2*direction)*40; //Constant correction
		bearing_prox = (1-2*direction)*100*(1-dist_mm/PROX_DIST_MIN); //Linear correction
		return bearing_prox;
	}

	if(obstacle_cleared)
		return 0;
	else{
		if(switch_direction_flag){
			direction = !direction;
			switch_direction_flag = 0;
		}
		bearing_prox = (1-2*!direction)*(fabs(bearing_prox)-PROX_DEC_COEFF); //Decrement bearing_prox
		if(fabs(bearing_prox) <= PROX_DEC_COEFF){ //Case bearing_prox cannot be decremented without changing sign
			obstacle_cleared = 1;
			bearing_prox = 0;
		}
		return bearing_prox;
	}
}

void move (int16_t bearing){
	static int16_t bearing_prev = 0; //Used for D term
	static int16_t bearingI = 0; //Used for I term

	if(fabs(bearingI) < 1 || (bearingI >= 1 && bearing < 0) || (bearingI <= -1 && bearing > 0)) //Prevent saturation
		bearingI += bearing ;

	int16_t delta = Kp*bearing + Kd*(bearing - bearing_prev)+ Ki*bearingI;
	if(delta > DELTA_MAX) //Limit maximal acceleration
		delta = DELTA_MAX;
	if(delta < DELTA_MIN)
		delta = DELTA_MIN;

	bearing_prev = bearing;

	left_motor_set_speed(SPEED_BASE - SPEED_MAX_COEFF*MOTOR_SPEED_LIMIT*delta);
	right_motor_set_speed(SPEED_BASE + SPEED_MAX_COEFF*MOTOR_SPEED_LIMIT*delta);

	climby_leds_handler(MOVEMENT, delta);
	//Led handling done here because real correction value (delta) does not necessarily correspond to bearing
}

static THD_WORKING_AREA(waSetPath, 512);

static THD_FUNCTION(SetPath, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(climby_calibrate_acc()){
    	climby_leds_handler(CALIBRATION,0);
    }

    //systime_t time;

    while(1){
    	systime_t time = chVTGetSystemTime(); //IS THIS ALLOWED ???

        uint16_t ToF_dist_mm = VL53L0X_get_dist_mm();

        int16_t acc[3] = {0};
        get_averaged_acc(acc);

    	if((acc[X_AXIS] < IMU_TOP_MAX_X && acc[X_AXIS] > IMU_TOP_MIN_X) &&
    		(acc[Y_AXIS] < IMU_TOP_MAX_Y && acc[Y_AXIS] > IMU_TOP_MIN_Y) &&
    	    (acc[Z_AXIS] < IMU_TOP_MAX_Z && acc[Z_AXIS] > IMU_TOP_MIN_Z)){ //Top reached
    		left_motor_set_speed(0);
    		right_motor_set_speed(0);
    		climby_leds_handler(TOP_REACHED,0);
    	}
         else{
        	 int16_t bearing_prox = prox_bearing(ToF_dist_mm);
        	 int16_t bearing_imu = imu_bearing(acc[X_AXIS], acc[Y_AXIS], acc[Z_AXIS]);
        	 move((1-bearing_prox/100)*bearing_imu + bearing_prox);
         }

    	chThdSleepUntilWindowed(time, time + MS2ST(10)); //100 Hz
    }
}

void set_path_start(void){
	chThdCreateStatic(waSetPath, sizeof(waSetPath), NORMALPRIO, SetPath, NULL);
}

//OLDER VARIANT I REMOVED :

/*
static THD_FUNCTION(SetPath, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    int16_t offset_x;
    int16_t offset_y;
    int16_t offset_z;
    do{
    	blink_leds(0, 0);
    	calibrate_acc(); //Collects samples for calibration
    	offset_x = get_acc_offset(X_AXIS);
    	offset_y = get_acc_offset(Y_AXIS);
    	offset_z = get_acc_offset(Z_AXIS);
    }while(offset_z < IMU_OFFSET_MAX || offset_z > IMU_OFFSET_MIN);
    //Protection in case of calibration with the robot tilted

    systime_t time;

    int16_t acc_x_buffer[IMU_BUFFER_SIZE_XY] = {0};
    int16_t acc_y_buffer[IMU_BUFFER_SIZE_XY] = {0};
    int16_t acc_z_buffer[IMU_BUFFER_SIZE_Z] = {0};
    int buffer_place_xy = 0;
    int buffer_place_z = 0;
    int32_t acc_x_sum=0; //Used to calculate a running average
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
    	//chprintf((BaseSequentialStream *)&SD3, "Acc_X = %d \r\n", acc_x_averaged);
    	//chprintf((BaseSequentialStream *)&SD3, "Acc_Y = %d \r\n", acc_y_averaged);
    	//chprintf((BaseSequentialStream *)&SD3, "Acc_Z = %d \r\n", acc_z_averaged);

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
    	}
         else{
        	 bearing_prox = prox_bearing(ToF_dist_mm);
        	 chprintf((BaseSequentialStream *)&SD3, "Bearing_PROX = %d", bearing_prox);
        	 bearing_imu = imu_bearing(acc_x_averaged, acc_y_averaged, acc_z_averaged);
        	 chprintf((BaseSequentialStream *)&SD3, "Bearing_IMU = %d", bearing_imu);
        	 bearing = (1-bearing_prox/100)*bearing_imu + bearing_prox;
        	 move(bearing);
         }

    	chThdSleepUntilWindowed(time, time + MS2ST(10)); //100 Hz
    }
}*/
