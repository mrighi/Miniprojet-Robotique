#ifndef CLIMB_H
#define CLIMB_H

#define PROX_FRONT_RIGHT 			0
#define PROX_FRONT_LEFT 			7
#define PROX_DIAG_RIGHT				1
#define PROX_DIAG_LEFT				6
#define PROX_LEFT					5
#define PROX_RIGHT					2

#define X_AXIS						0
#define Y_AXIS						1
#define Z_AXIS						2

#define IMU_SAMPLE_SIZE				50 		//The base functions also use 50 idk
#define PLATEAU_DETECTION_THRESHOLD	0.01	//Determined empirically

#define PROX_THRESHOLD				1000 	//Determined empirically

//Start the SetPath thread
void set_path_start(void);

#endif /* CLIMB_H */
