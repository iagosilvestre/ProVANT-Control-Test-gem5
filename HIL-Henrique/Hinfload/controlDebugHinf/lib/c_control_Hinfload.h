/**
  ******************************************************************************
  * @file    modules/control/c_control_HinfLoad.h
  * @author  Arthur Viana Lara
  * @version V1.0.0
  * @date    28-Novembro-2017
  * @brief   Controle de rastreamento de trajetória por realimentacao de estados 
  *	     calculado com a estratégia de controle misto Hinfinito/H2.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_CONTROL_HINFLOAD_H
#define C_CONTROL_HINFLOAD_H

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
void c_control_HinfLoad_init();

pv_type_actuation c_control_HinfLoad_controller(pv_msg_input input);

#ifdef __cplusplus
}
#endif

#endif 
