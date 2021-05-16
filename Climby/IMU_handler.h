#ifndef IMU_HANDLER_H_
#define IMU_HANDLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#define X_AXIS						0
#define Y_AXIS						1
#define Z_AXIS						2

#define IMU_OFFSET_MAX				-15000		//Expected z-direction offset range
#define IMU_OFFSET_MIN				-17000

#define IMU_BUFFER_SIZE_XY			5			//Number of acceleration samples taken
#define IMU_BUFFER_SIZE_Z			50

/**
* @brief	Calculates the accelerometer offsets
* 			Used as a condition for a while loop to
* 				ensure calibration is done with the robot on a flat surface
*
* @return	1 if the acceleration value isn't within a given range, 0 otherwise
*/
bool climby_calibrate_acc(void);

/**
* @brief	Calculates running average of the most recent accelerometer measures on all axes
*
* @param	Pointer to an array of size (at least) 3 to store the calculated values in
*
* @return	Averaged acceleration values
*/
void get_averaged_acc(int16_t* acc);

#endif /* IMU_HANDLER_H_ */
