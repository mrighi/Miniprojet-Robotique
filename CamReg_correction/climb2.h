#ifndef CLIMB2_H_
#define CLIMB2_H_

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

#define SPEED_BASE					300		//Base movement speed
#define SPEED_INC_COEFF				1
#define SPEED_MAX_COEFF				0.3 	//Limit max speed of the robot

#define IMU_SAMPLE_SIZE				50 		//The base functions use 50
#define IMU_EPSILON					0.01	//Determined empirically
#define IMU_RESOLUTION				32000
#define IMU_MAX						2*g

#define PROX_OFFSET_MAX				500
#define PROX_MAX					1000
#define PROX_THRESHOLD				60 		//Determined empirically

#define Kp							0.3
#define Kd							0.3
#define Ki							0.3

//#define COEFF_IMU					0.3		//Determined empirically
//#define COEFF_PROX					0.7		//Determined empirically


int8_t imu_bearing(int16_t acc_x, int16_t acc_y);

int8_t prox_bearing(int prox_front_left, int prox_front_right, int prox_diag_left, int prox_diag_right);

void move(float bearing);

void set_path_start(void);

#endif /* CLIMB2_H_ */
