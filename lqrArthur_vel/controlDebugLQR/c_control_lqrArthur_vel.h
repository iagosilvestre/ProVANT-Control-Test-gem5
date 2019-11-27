/**
  ******************************************************************************
  * @file    modules/control/c_control_LQR.h
  * @author  Iuro Nascimento
  * @version V1.0.0
  * @date    28-July-2016
  * @brief   Controlador LQR baseado em c_rc_LQR_control.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_CONTROL_LQRARTHUR_VEL_H
#define C_CONTROL_LQRARTHUR_VEL_H

#define ARM_MATH_CM4
#include "arm_math.h"
#include "c_rc_commons.h"
#include "pv_typedefs.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void c_control_lqrArthur_vel_init();

//LQR attitude and height(AH) controller. Height control depends on global variable manual_height_control.
pv_type_actuation c_control_lqrArthur_vel_controller(pv_msg_input inputData);

#ifdef __cplusplus
}
#endif

#endif 
