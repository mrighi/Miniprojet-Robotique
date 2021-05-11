#ifndef CLIMB_H
#define CLIMB_H

#define STATIC_PROX

#define PROX_FRONT_RIGHT 			0
#define PROX_FRONT_LEFT 			7
#define PROX_DIAG_RIGHT				1
#define PROX_DIAG_LEFT				6
#define PROX_LEFT					5
#define PROX_RIGHT					2

#define X_AXIS						0
#define Y_AXIS						1
#define Z_AXIS						2

#define g							9.81

#define SPEED						550 	//Goes -1100 to 1100
#define SPEED_MAX					1100

//#define IMU_SAMPLE_SIZE				50 		//The base functions use 50
#define IMU_EPSILON					0.01	//Determined empirically

#define IMU_RESOLUTION				32000
#define IMU_MAX						2*g

#define PROX_THRESHOLD_FRONT		60 	//Determined empirically
#define PROX_THRESHOLD_DIAG			60 	//Determined empirically //Should be smaller than front

#define COEFF_IMU					0.6		//Determined empirically
#define COEFF_PROX					0.4		//Determined empirically


void turn_left(void);

void turn_right(void);

void stop_motors(void);

void go_straight(void);


float imu_bearing(int16_t acc_x, int16_t acc_y);

void angle_motor_treatment(float angle);


void set_path_start(void);

#endif /* CLIMB_H */
