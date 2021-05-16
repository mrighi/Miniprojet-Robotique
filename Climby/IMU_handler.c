#include <sensors/imu.h>
#include <IMU_handler.h>

static int16_t offset_x = 0;
static int16_t offset_y = 0;
static int16_t offset_z = 0;

bool climby_calibrate_acc(void){
	    calibrate_acc(); //Collects samples for calibration
	    offset_x = get_acc_offset(X_AXIS);
	    offset_y = get_acc_offset(Y_AXIS);
	    offset_z = get_acc_offset(Z_AXIS);
	    return offset_z > IMU_OFFSET_MAX || offset_z < IMU_OFFSET_MIN; //Verify the robot is calibrated on its "stomach"
}

void get_averaged_acc(int16_t* acc){
	static int16_t acc_x_buffer[IMU_BUFFER_SIZE_XY] = {0};
	static int16_t acc_y_buffer[IMU_BUFFER_SIZE_XY] = {0};
	static int16_t acc_z_buffer[IMU_BUFFER_SIZE_Z] = {0};
	static int8_t buffer_place_xy = 0;
	static int8_t buffer_place_z = 0;
	static int32_t acc_x_sum=0; //Used to calculate a running average
	static int32_t acc_y_sum=0;
	static int32_t acc_z_sum=0;

	//Subtract the oldest acceleration values and add the latest to the running sum
	acc_x_sum -= acc_x_buffer[buffer_place_xy];
	acc_y_sum -= acc_y_buffer[buffer_place_xy];
	acc_z_sum -= acc_z_buffer[buffer_place_z];
	acc_x_buffer[buffer_place_xy]= get_acc(X_AXIS)-offset_x ;
	acc_y_buffer[buffer_place_xy]= get_acc(Y_AXIS)-offset_y ;
	acc_z_buffer[buffer_place_z]= get_acc(Z_AXIS)-offset_z ;
	acc_x_sum += acc_x_buffer[buffer_place_xy];
	acc_y_sum += acc_y_buffer[buffer_place_xy];
	acc_z_sum += acc_z_buffer[buffer_place_z];

	acc[X_AXIS] = acc_x_sum / IMU_BUFFER_SIZE_XY;
	acc[Y_AXIS] = acc_y_sum / IMU_BUFFER_SIZE_XY;
	acc[Z_AXIS] = acc_z_sum / IMU_BUFFER_SIZE_Z;

	buffer_place_xy = (buffer_place_xy + 1) % IMU_BUFFER_SIZE_XY ; //Increment position on the buffers
	buffer_place_z = (buffer_place_z + 1) % IMU_BUFFER_SIZE_Z ;
}
