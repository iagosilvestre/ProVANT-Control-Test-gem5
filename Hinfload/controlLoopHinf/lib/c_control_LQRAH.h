/**
  ******************************************************************************
  * @file    modules/control/c_control_LQR_control.h
  * @author  Rodrigo Donadel
  * @version V1.0.0
  * @date    08-December-2014
  * @brief   Controle de estabilizacao por realimentacao de estados calculado com LQR.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_CONTROL_LQRAH_H
#define C_CONTROL_LQRAH_H

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
void c_control_LQRAH_init();

//LQR attitude and height(AH) controller. Height control depends on global variable manual_height_control.
pv_type_actuation c_control_LQRAH_controller(pv_type_datapr_attitude attitude,
				  pv_type_datapr_attitude attitude_reference,
				  pv_type_datapr_position position,
				  pv_type_datapr_position position_reference,
				  float throttle_control,
				  bool manual_height_control);

#ifdef __cplusplus
}
#endif

#endif //C_control_SFC_H
