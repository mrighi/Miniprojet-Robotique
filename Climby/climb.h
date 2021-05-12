#ifndef CLIMB_H_
#define CLIMB_H_

#ifdef __cplusplus
extern "C" {
#endif

//#define X_AXIS						0
//#define Y_AXIS						1
//#define Z_AXIS						2

//#define g							9.81

#define BEARING_MAX					100

#define SPEED_BASE					500		//Base movement speed
#define BEARING_TO_SPEED			0.003*1100 	//Empirical

#define DELTA_SPEED_MIN				-800
#define DELTA_SPEED_MAX 			600 //Motor saturates before +800

//#define IMU_BUFFER_SIZE_XY			5
//#define IMU_BUFFER_SIZE_Z			50

#define IMU_GO_STRAIGHT_THRESHOLD	320

//#define IMU_EPSILON					0.01	//Determined empirically
//#define IMU_RESOLUTION				32000
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
int8_t imu_bearing(int16_t acc_x, int16_t acc_y);

/**
* @brief	Used to find the rotational correction value to avoid obstacles
* 			Chooses an arbitrary direction which is toggled  at every subsequent obstacle
*
* @param	ToF sensor distance value in mm
*
* @return	A bearing value [-100,100] proportional to the distance between
* 				the robot and an obstacle less than 10cm away and of arbitrary sign
*/
int8_t prox_bearing(uint16_t dist_mm);

/**
* @brief	PI controller
*
* @param	A bearing value [-100,100]
*/
void move(int8_t bearing);

/**
* @brief	Start the movement thread : collects accelerometer and ToF sensor values and
* 				sets the motor speeds accordingly
*/
void set_path_start(void);

#endif /* CLIMB_H_ */
