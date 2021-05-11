#ifndef CLIMB_H_
#define CLIMB_H_

#ifdef __cplusplus
extern "C" {
#endif

//#define X_AXIS						0
//#define Y_AXIS						1
//#define Z_AXIS						2

//#define g							9.81

#define SPEED_BASE					500		//Base movement speed
#define SPEED_MAX_COEFF				0.003 	//Limit max speed of the robot

#define DELTA_MAX 					250
#define DELTA_MIN					-DELTA_MAX

//#define IMU_BUFFER_SIZE_XY			5
//#define IMU_BUFFER_SIZE_Z			50

#define IMU_EPSILON					0.01	//Determined empirically
#define IMU_RESOLUTION				32000
//#define IMU_MAX						2*g
//#define IMU_OFFSET_MAX				-17000
//#define IMU_OFFSET_MIN				-15000

#define IMU_TOP_MAX_X				400
#define IMU_TOP_MIN_X				-400
#define IMU_TOP_MAX_Y				400
#define IMU_TOP_MIN_Y				-900
#define IMU_TOP_MAX_Z				400
#define IMU_TOP_MIN_Z				-400

//#define PROX_OFFSET_MAX				500
//#define PROX_MAX					1000
//#define PROX_THRESHOLD				60 		//Determined empirically

#define PROX_DIST_MIN				100
#define PROX_DEC_COEFF				2

#define Kp							2
#define Kd							0
#define Ki							0.2

/**
* Convention used throughout for bearings: trigonometric direction
* (0,100] = left, 0 = straight, [-100, 0) = right
*/

 /**
 * @brief	Used to find the rotational correction value to maximize climb
 *
 * @param	Accelerometer values from the three axes
 * 			Assumes x,y,z acceleration is zero when the robot is parallel to the ground
 *
 * @return	A bearing value [-100,100] proportional to the angle between
 * 				the robot's direction of movement and the axis of maximal climb
 */
int16_t imu_bearing(int32_t acc_x, int32_t acc_y, int32_t acc_z);

/**
* @brief	Used to find the rotational correction value to avoid obstacles
* 			Chooses an arbitrary direction which is toggled  at every subsequent obstacle
*
* @param	ToF sensor distance value in mm
*
* @return	A bearing value [-100,100] proportional to the distance between
* 				the robot and an obstacle less than 10cm away and of arbitrary sign
*/
int16_t prox_bearing(uint16_t dist_mm);

/**
* @brief	PI controller
*
* @param	A bearing value [-100,100]
*/
void move(int16_t bearing);

/**
* @brief	Start the movement thread : collects accelerometer and ToF sensor values and
* 				sets the motor speeds accordingly
*/
void set_path_start(void);

#endif /* CLIMB_H_ */
