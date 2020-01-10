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
#include "c_control_lqrArthur_vel.c"
#include "c_control_lqrArthur_vel.h"
#define ARM_MATH_CM4
#include "lib/arm_mat_init_f32.c"
#include "lib/arm_mat_sub_f32.c"
#include "lib/arm_mat_mult_f32.c"
#include "lib/arm_math.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MODULE_PERIOD	 12//ms
#define ESC_ON           1
#define SERVO_ON         1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//portTickType lastWakeTime;
pv_msg_input iInputData;
pv_msg_controlOutput oControlOutputData;
pv_type_datapr_position iRefData;
pv_type_datapr_position pos_ref;

/* Inboxes buffers */
pv_type_actuation    iActuation;
/* Outboxes buffers*/

void module_co_init()
{

  c_control_lqrArthur_vel_init();
  std::cout << "c_control_lqrArthur_vel_init ended" << std::endl;
  pos_ref.x = 2.0;
  pos_ref.y = 0.0;
  pos_ref.z = 1.5;

	/* Inicializa os dados da attitude*/
	oControlOutputData.actuation.servoRight = 0;
	oControlOutputData.actuation.servoLeft  = 0;
	oControlOutputData.actuation.escRightNewtons = 10.2751;
	oControlOutputData.actuation.escLeftNewtons = 10.2799;
}

void module_co_run()
{



		iInputData.position_reference.x = pos_ref.x;
		iInputData.position_reference.y = pos_ref.y;
		iInputData.position_reference.z = pos_ref.z;
		iInputData.position_reference.dotX = pos_ref.dotX;
		iInputData.position_reference.dotY = pos_ref.dotY;
		iInputData.position_reference.dotZ = pos_ref.dotZ;
//		iInputData.position.x = pos_ref.x;
//		iInputData.position.y = pos_ref.y;
//		iInputData.position.z = pos_ref.z;
//		iInputData.position.dotX = pos_ref.dotX;
//		iInputData.position.dotY = pos_ref.dotY;
//		iInputData.position.dotZ = pos_ref.dotZ;
		oControlOutputData.actuation = c_control_lqrArthur_vel_controller(iInputData);

	}


int main()
{

		std::cout << "module init starting" << std::endl;
		m5_reset_stats(0,0);
		module_co_init();
		m5_dump_stats(0,0);
		std::cout << "module init ended" << std::endl;

		std::cout << "module run starting" << std::endl;
		m5_reset_stats(0,0);
		module_co_run();
		m5_dump_stats(0,0);
		std::cout << "module run ended" << std::endl;

    return 0;
}
