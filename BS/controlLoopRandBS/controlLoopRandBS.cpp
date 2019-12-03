/*
Provided to you by Emlid Ltd (c) 2014.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Get pressure from MS5611 barometer onboard of Navio shield for Raspberry Pi

To run this example navigate to the directory containing it and run following commands:
make
./Barometer
*/

#define _GNU_SOURCE
#include <sched.h>
//#include <Common/Ublox.h>
#include <string>
#include <stdio.h>
#include <memory>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
//#include <Common/Util.h>
#include <pthread.h>
#include <iostream>
#include <unistd.h>
#include <vector>



#include "m5op.h"


#include "pv_module_co.h"
//#include "c_control_lqrArthur.h"
#include "pv_typedefs.h"


extern "C" {
#include "lib/c_control_BS.c"
#include "lib/c_control_BS.h"

}

#define ARM_MATH_CM4
#include "lib/arm_mat_init_f32.c"
#include "lib/arm_mat_sub_f32.c"
#include "lib/arm_mat_mult_f32.c"
#include "lib/arm_mat_add_f32.c"
#include "lib/arm_math.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MODULE_PERIOD	 12//ms
#define ESC_ON           1
#define SERVO_ON         1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//portTickType lastWakeTime;
extern "C" {
pv_type_datapr_attitude attitude;
pv_type_datapr_attitude attitude_reference;
pv_type_datapr_position position;
pv_type_datapr_position pos_ref;
pv_msg_input iInputData;
pv_msg_controlOutput oControlOutputData;
}

/* Inboxes buffers */
pv_type_actuation    iActuation;
/* Outboxes buffers*/

void module_co_init()
{

	c_control_BS_init();
	pos_ref.x = 2.0;
	pos_ref.y = 0.0;
	pos_ref.z = 1.5;
	pos_ref.dotX = 0;
	pos_ref.dotY = 0;
	pos_ref.dotZ = 0;

	attitude_reference.roll=0;
	attitude_reference.pitch=0;
	attitude_reference.yaw=0;
	attitude_reference.dotRoll=0;
	attitude_reference.dotPitch=0;
	attitude_reference.dotYaw=0;

	/* Inicializa os dados da attitude*/
	oControlOutputData.actuation.servoRight = 0;
	oControlOutputData.actuation.servoLeft  = 0;
	oControlOutputData.actuation.escRightNewtons = 10.2751;
	oControlOutputData.actuation.escLeftNewtons = 10.2799;
}

void module_co_run()
{

		pos_ref.x = rand() % 10 + 1;
		pos_ref.y = rand() % 10 + 1;
		pos_ref.z = rand() % 10 + 1;
		pos_ref.dotX = rand() % 10 + 1;
		pos_ref.dotY = rand() % 10 + 1;
		pos_ref.dotZ = rand() % 10 + 1;

		attitude_reference.roll = rand() % 4 + 1;
		attitude_reference.pitch = rand() % 4 + 1;
		attitude_reference.yaw = rand() % 4 + 1;
		attitude_reference.dotRoll = rand() % 4 + 1;
		attitude_reference.dotPitch = rand() % 4 + 1;
		attitude_reference.dotYaw = rand() % 4 + 1;


//		iInputData.position_reference.x = pos_ref.x;
//		iInputData.position_reference.y = pos_ref.y;
//		iInputData.position_reference.z = pos_ref.z;
//		iInputData.position_reference.dotX = pos_ref.dotX;
//		iInputData.position_reference.dotY = pos_ref.dotY;
//		iInputData.position_reference.dotZ = pos_ref.dotZ;


//		iInputData.position.x = pos_ref.x;
//		iInputData.position.y = pos_ref.y;
//		iInputData.position.z = pos_ref.z;
//		iInputData.position.dotX = pos_ref.dotX;
//		iInputData.position.dotY = pos_ref.dotY;
//		iInputData.position.dotZ = pos_ref.dotZ;
		m5_reset_stats(0,0);
		oControlOutputData.actuation = c_control_BS_AH_controller(attitude,attitude_reference,position,pos_ref,0,false,true);

	}


int main()
{
	int heartBeat=0;
	//m5_reset_stats(0,0); // Reseta os stats quando inicializa de fato o programa.
	module_co_init();
	m5_dump_stats(0,0); //Posta stats após inicialização do módulo de controle
	while(heartBeat<=100){
		heartBeat+=1;

//		m5_reset_stats(0,0);

		position.x = pos_ref.x;
		position.y = pos_ref.y;
		position.z = pos_ref.z;
		position.dotX = pos_ref.dotX;
		position.dotY = pos_ref.dotY;
		position.dotZ = pos_ref.dotZ;

		attitude.roll = attitude_reference.roll;
		attitude.pitch = attitude_reference.pitch;
		attitude.yaw = attitude_reference.yaw;
		attitude.dotRoll = attitude_reference.dotRoll;
		attitude.dotPitch = attitude_reference.dotPitch;
		attitude.dotYaw = attitude_reference.dotYaw;

		module_co_run();
		m5_dump_stats(0,0); // Posta stats novamente após término do modulo de controle
	}
    return 0;
}
