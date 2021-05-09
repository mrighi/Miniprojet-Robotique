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

#define SPEED_BASE					500		//Base movement speed
#define SPEED_INC_COEFF				1
#define SPEED_MAX_COEFF				0.003 	//Limit max speed of the robot

#define IMU_BUFFER_SIZE_XY			5
#define IMU_BUFFER_SIZE_Z			50
#define IMU_SAMPLE_SIZE				100 	//The base functions use 50
#define IMU_EPSILON					0.01	//Determined empirically
#define IMU_RESOLUTION				32000
#define IMU_MAX						2*g
#define IMU_OFFSET_MAX				-17000
#define IMU_OFFSET_MIN				-15000

#define IMU_TOP_MAX_X				400
#define IMU_TOP_MIN_X				-400
#define IMU_TOP_MAX_Y				400
#define IMU_TOP_MIN_Y				-900
#define IMU_TOP_MAX_Z				400
#define IMU_TOP_MIN_Z				-400

#define PROX_OFFSET_MAX				500
#define PROX_MAX					1000
#define PROX_THRESHOLD				60 		//Determined empirically
#define PROX_DEC_COEFF				1

#define Kp							2
#define Kd							1
#define Ki							0

//#define COEFF_IMU					0.3		//Determined empirically
//#define COEFF_PROX					0.7		//Determined empirically


int16_t imu_bearing(int32_t acc_x, int32_t acc_y, int32_t acc_z);

//int16_t prox_bearing(int prox_front_left, int prox_front_right, int prox_diag_left, int prox_diag_right);
int16_t prox_bearing(uint16_t dist_mm);

void move(int16_t bearing);

void set_path_start(void);

#endif /* CLIMB2_H_ */
