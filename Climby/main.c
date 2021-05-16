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
#include <sensors/VL53L0X/VL53L0X.h> //ToF
#include <spi_comm.h>

#include <climb.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

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
	mpu_init();

	//Serial communication initialization
	serial_start();
	usb_start(); //is this necessary ?

	//Motors initialization
	motors_init();

	//Inter Process Communication bus initialization
	messagebus_init(&bus, &bus_lock, &bus_condvar);

	//IMU initialization
	imu_start();

	//ToF sensor initialization
	VL53L0X_start();

	//Comm for RGB LEDs initialization
	spi_comm_start(); //Used to set RGB leds

	//Start the SetPath thread
	set_path_start();

    //Infinite loop
    while (1) {

    }

	return 0;
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
