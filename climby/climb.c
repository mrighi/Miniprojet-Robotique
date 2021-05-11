//MUST CORRECT:
//MAX_SPEED is already in motors.h
//Use filtered accelerometer values
//See if it's possible to optimize the giant if tree
//Remove if cases where the motor doesn't need to be updated
//In this form, it's redundant to use atan because we only turn left or right

//POSSIBLE CORRECTIONS
//Swap the diag it sensors for the side sensors

#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <i2c_bus.h>

#include <main.h> //COPIED FROM TP5 ; I HAVE MY DOUBTS
#include <motors.h>
#include <sensors/imu.h>
#include <sensors/proximity.h>
#include <climb.h>

static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

void turn_left(void){
	left_motor_set_speed(SPEED_MAX);
	right_motor_set_speed(-SPEED_MAX);
}

void turn_right(void){
	left_motor_set_speed(SPEED_MAX);
	right_motor_set_speed(-SPEED_MAX);
}

void stop_motors(void){
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void go_straight(void){
	left_motor_set_speed(SPEED); //Not set to max because I don't want to break the robot
	right_motor_set_speed(SPEED); //Also might be more sensitive at higher speed
}

//Using double because idk how to get arctan in float SHOULD BE CHANGED !!!
//Using degrees to be able to use ints
//Angles are in trigonometric direction
float imu_bearing(int16_t acc_x, int16_t acc_y){
	//Limit cases to avoid division by zero
	if(acc_y <= IMU_EPSILON*2/IMU_RESOLUTION && acc_x > 0){
		return 90 ;
	}
	if(acc_y <= IMU_EPSILON*2/IMU_RESOLUTION && acc_x < 0){
		return -90 ;
	}
	return 57.3f*atan2(acc_x,acc_y);
	//57.3 is to convert to degree
	//f so it's compiled as a float
}

void angle_motor_treatment(float angle){
	if(angle<0){
		turn_left();
	}
	if(angle>0){
		turn_right();
	}
	if(angle == 0){
		go_straight();
	}
}

static THD_WORKING_AREA(waSetPath, 256);

static THD_FUNCTION(SetPath, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    int16_t offset_x = 0;
    int16_t offset_y = 0;
    int16_t offset_z = 0;

    //Protection in case of calibration with the sensors covered
    do{
    	calibrate_acc(); //Collects samples for calibration
    	offset_x = get_acc_offset(X_AXIS);
    	offset_y = get_acc_offset(Y_AXIS);
    	offset_z = get_acc_offset(Z_AXIS);
    }while((offset_x >= 500) || (offset_y >= 500) || (offset_z >= 500));

    systime_t time;

    int16_t acc_x_calibrated;
    int16_t acc_y_calibrated;
    int16_t acc_z_calibrated;

    int prox_front_left;
    int prox_front_right;
    int prox_diag_left;
    int prox_diag_right;
    //int prox_left;
    //int prox_right;

    bool coll_front_left;
    bool coll_front_right;
    bool coll_diag_left;
    bool coll_diag_right;
    //bool coll_left;
    //bool coll_right;

    float imu_angle = 0;
    float imu_angle_prev; //used to help treat zero case

    //I don't think these need to be static
    //MAY BE A SOURCE OF ERRORS
    bool avoiding_coll_left = 0 ;
    bool avoiding_coll_right = 0;

    bool stuck_toggle = 0; //Treats the case where it hits an obstacle head-on

    while(1){
    	time = chVTGetSystemTime();

//Collect and print all values
    	//It's redundant to assign memory space unless variables are called more than once
    	//Done here because all variables are also printed
    	//Remove for final version

    	//Collect acceleration values
    	acc_x_calibrated=(get_acceleration(X_AXIS)-offset_x);
    	acc_y_calibrated=(get_acceleration(Y_AXIS)-offset_y);
    	acc_z_calibrated=(get_acceleration(Z_AXIS)-offset_z);

    	//Print the offsets:
    	chprintf((BaseSequentialStream *)&SD3, "Offset_X = %d \r\n", offset_x);
    	chprintf((BaseSequentialStream *)&SD3, "Offset_Y = %d \r\n", offset_y);
    	chprintf((BaseSequentialStream *)&SD3, "Offset_Z = %d \r\n", offset_z);

    	//Print the accelerometer values:
    	chprintf((BaseSequentialStream *)&SD3, "Acc_X = %d \r\n", acc_x_calibrated);
    	chprintf((BaseSequentialStream *)&SD3, "Acc_Y = %d \r\n", acc_y_calibrated);
    	chprintf((BaseSequentialStream *)&SD3, "Acc_Z = %d \r\n", acc_z_calibrated);

    	//Collect the proximity values
    	prox_front_left = get_calibrated_prox(PROX_FRONT_LEFT);
    	prox_front_right = get_calibrated_prox(PROX_FRONT_RIGHT);
    	prox_diag_left = get_calibrated_prox(PROX_DIAG_LEFT);
    	prox_diag_right = get_calibrated_prox(PROX_DIAG_RIGHT);
    	//prox_left = get_calibrated_prox(PROX_LEFT);
        //prox_right = get_calibrated_prox(PROX_RIGHT);

        //Print proximity values
        chprintf((BaseSequentialStream *)&SD3, "Prox_FL = %d \r\n", prox_front_left);
        chprintf((BaseSequentialStream *)&SD3, "Prox_FR = %d \r\n", prox_front_right);
        chprintf((BaseSequentialStream *)&SD3, "Prox_DL = %d \r\n", prox_diag_left);
        chprintf((BaseSequentialStream *)&SD3, "Prox_DR = %d \r\n", prox_diag_right);
        //chprintf((BaseSequentialStream *)&SD3, "Prox_L = %d \r\n", prox_left);
        //chprintf((BaseSequentialStream *)&SD3, "Prox_R = %d \r\n", prox_right);

        //Collect collisions data
        coll_front_left = prox_front_left >= PROX_THRESHOLD_FRONT; // 1 = collision
        coll_front_right = prox_front_left >= PROX_THRESHOLD_FRONT;
        coll_diag_left = prox_front_left >= PROX_THRESHOLD_DIAG;
        coll_diag_right = prox_front_left >= PROX_THRESHOLD_DIAG;
        //coll_left = prox_front_left >= PROX_THRESHOLD;
        //coll_right = prox_front_left >= PROX_THRESHOLD;

       //Print collision values
        chprintf((BaseSequentialStream *)&SD3, "Coll_FL = %d \r\n", coll_front_left);
        chprintf((BaseSequentialStream *)&SD3, "Coll_FR = %d \r\n", coll_front_right);
        chprintf((BaseSequentialStream *)&SD3, "Coll_DL = %d \r\n", coll_diag_left);
        chprintf((BaseSequentialStream *)&SD3, "Coll_DR = %d \r\n", coll_diag_right);
        //chprintf((BaseSequentialStream *)&SD3, "Coll_L = %d \r\n", coll_left);
        //chprintf((BaseSequentialStream *)&SD3, "Coll_R = %d \r\n", coll_right);

        //Maximal pitch angle
        imu_angle_prev = imu_angle;
        imu_angle=imu_bearing(acc_x_calibrated, acc_y_calibrated);

        //Print imu angle
        chprintf((BaseSequentialStream *)&SD3, "IMU_angle = %.4f \r\n", imu_angle);

//Movement algorithm:
//The robot usually follows the imu angle
//In case of obstacle, it turns until there is no obstacle straight ahead,
        //then goes straight until the diagonal sensors are clear
        //then goes back to normal function
//Rotation is only updated once prox sensors are cleared

//Particular cases:
        //If both sensors blocked, follows imu angle direction
        //If both sensors blocked and imu direction is zero then alternates either left or right
//This last case is important ; think L-shaped box hit head-on

//Giant if-else of all possible cases
//Has some overlaps in motor settings but not in flag settings

        //Case top reached
        if(acc_z_calibrated <= -(1 - IMU_EPSILON)*2/IMU_RESOLUTION){ //Minus because z axis points up // 2/res corresponds to g
           stop_motors();
           chprintf((BaseSequentialStream *)&SD3, "TOP REACHED");
        }

        //Case no previous collisions
        else if(! avoiding_coll_left && !avoiding_coll_right){
        	if(!coll_front_left && !coll_front_right){ //No current collisions
        		angle_motor_treatment(imu_angle);
        	}
        	else if(coll_front_left){ //Collision left
        		avoiding_coll_left = 1;
        		turn_right();
        	}
        	else if(coll_front_right){ //collision right
        	    avoiding_coll_right = 1;
        	    turn_left();
        	}
        	else if(coll_front_left && coll_front_right){ //Head-on collision
        		avoiding_coll_right = 1;
        		avoiding_coll_left = 1;
        		if(imu_angle != 0){
        			angle_motor_treatment(imu_angle);
        		}
        		else{ //treats the case where it hit an obstacle aligned with optimal climb angle
        			if(stuck_toggle){ //tries to go one way, next it'll try the next
        				++stuck_toggle;
        				turn_left();
        			}
        			else{
        				++stuck_toggle;
        				turn_right();
        			}
        		}
        	}
        }

        //Case avoiding collision on left
        else if(avoiding_coll_left){
        	//if(coll_front_left){ //Pretty sure this is redundant
        		//turn_right();
        	//}
        	if(coll_front_right){ //Becomes a head-on collision
        		avoiding_coll_right = 1;
        		 if(imu_angle != 0){
        			 angle_motor_treatment(imu_angle);
        		 }
        		 else{ //treats the case where it hit an obstacle aligned with optimal climb angle
        		     if(stuck_toggle){ //tries to go one way, next it'll try the next
        		    	 ++stuck_toggle;
        		    	 turn_left();
        		     }
        		     else{
        		    	 ++stuck_toggle;
        		    	 turn_right();
        		     }
        		 }
        	}
        	else if(coll_diag_left){ //Diag sensor still get obstacle but front doesn't
        		go_straight();
        	}
        	else{ //Clears the obstacle
        		avoiding_coll_left = 0;
        		angle_motor_treatment(imu_angle);
        	}
        }

        //Case avoiding collision on right
        else if(avoiding_coll_right){
        	if(coll_front_left){ //Becomes a head-on collision
        		coll_front_left = 1;
        		if(imu_angle != 0){
        			angle_motor_treatment(imu_angle);
        		}
        		else{ //treats the case where it hit an obstacle aligned with optimal climb angle
        		    if(stuck_toggle){ //tries to go one way, next it'll try the next
        		    	++stuck_toggle;
        		    	turn_left();
        		    }
        		    else{
        		    	++stuck_toggle;
        		    	turn_right();
        		    }
        		}
        	}
        	//else if(coll_front_right){
        	//	turn_left(); //Probably redundant
        	//}
        	else if(coll_diag_right){ //Diag sensor still get obstacle but front doesn't
        		go_straight();
        	}
        	else{ //Clears the obstacle
        		avoiding_coll_right = 0;
        		angle_motor_treatment(imu_angle);
        	}
        }

        //Case avoiding head-on collision
        else{
        	if(coll_front_left || coll_front_left){ //Head-on collision remains
        		//In this case, usually keep turning
        		//Unless robot falls 90° below optimal axis
        		if(acc_y_calibrated <=  -IMU_EPSILON*2/IMU_RESOLUTION){ //Robot is 90° below optimal angle
        			//Rmk: minus because that way the robot is free to move perpendicular to an obstacle
        			if(acc_x_calibrated < 0){ //turning in right direction
        				turn_left();
        			}
        			else{
        				turn_right();
        			}
        		}
        	}
        	else if(acc_x_calibrated >= 0 && coll_diag_right){ //Diag sensor still get obstacle but front doesn't
        		go_straight();
        	}
        	else if(imu_angle_prev <= 0 && coll_diag_left){ //Diag sensor still get obstacle but front doesn't
        	    go_straight();
        	}
        	else{ //Clears the obstacle
        		avoiding_coll_left = 0;
        		avoiding_coll_right = 0;
        		angle_motor_treatment(imu_angle);
        	}
        }

    	chThdSleepUntilWindowed(time, time + MS2ST(10)); //100 Hz
    }
}

void set_path_start(void){
	chThdCreateStatic(waSetPath, sizeof(waSetPath), NORMALPRIO, SetPath, NULL);
}
