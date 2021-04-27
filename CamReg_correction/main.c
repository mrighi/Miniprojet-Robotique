#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <sensors/proximity.h>
#include <sensors/imu.h>
#include <camera/po8030.h>
#include <chprintf.h>

#include <climb.h>

#include <pi_regulator.h>
#include <process_image.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

//Comes from TP4_correction
static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void){
	//ChibiOS initialization
	halInit();
	chSysInit();
	//mpu_init(); //Used in example project

	//Serial communication initialization
	serial_start();

	//Motors initialization
	motors_init();

	//IR sensors initialization
	proximity_start();

	//I2C bus initialization
	//i2cStart(); //May or may not be necessary for IMU

	//IMU initialization
	imu_start();

	//From TP4:
	//timer11_start();

	//Inter Process Communication bus initialization
	messagebus_init(&bus, &bus_lock, &bus_condvar);

	//Start all threads here
	//Start the SetPath thread
	set_path_start();

    //Infinite loop
    while (1) {
    	//put in ChThreadSleep
    }

	return 0;
}

//Comes from TP4_correction
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
