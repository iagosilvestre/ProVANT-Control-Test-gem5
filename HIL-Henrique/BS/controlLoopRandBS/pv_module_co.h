/**
  ******************************************************************************
  * @file    modules/rc/pv_module_rc.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   Definição do módulo de controle + output.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PV_MODULE_CO_H
#define PV_MODULE_CO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* FreeRTOS kernel includes */
//#include "FreeRTOS.h"
//#include "queue.h"
//#include "task.h"

/* proVANT includes */
//#include "c_common_gpio.h"
//#include "c_common_i2c.h"
//#include "c_io_blctrl.h"
//#include "c_io_servos.h"
//#include "pv_module_in.h"
//#include "pv_module_do.h"


/*Control includes*/
//#include "c_rc_BS_control.h"
//#include "c_rc_LQR_control.h"
// #include "c_control_Hinfload.h"

#include "pv_typedefs.h"

/* Exported types ------------------------------------------------------------*/
struct pv_interface_co 
{
	unsigned int iInputData;
	unsigned int oControlOutputData;
	unsigned int iRefData;
} pv_interface_co;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
//#define ESC_MINIMUM_VELOCITY	10//esc set point value (0-255)
//#define ENABLE_SERVO
//#define ENABLE_ESC
/* Exported functions ------------------------------------------------------- */
void module_co_init();
void module_co_run();

#ifdef __cplusplus
}
#endif

#endif //PV_MODULE_CO_H
