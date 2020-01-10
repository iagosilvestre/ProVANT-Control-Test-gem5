/**
  ******************************************************************************
  * @file    modules/rc/c_rc_receiver.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   Implementação do receiver do controle de rádio manual.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_RC_RECEIVER_H
#define C_RC_RECEIVER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"

#include "c_common_utils.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define C_RC_CHANNEL_ROLL      0
#define C_RC_CHANNEL_PITCH     1
#define C_RC_CHANNEL_THROTTLE  2
#define C_RC_CHANNEL_YAW       3
#define C_RC_CHANNEL_A         4
#define C_RC_CHANNEL_B         5
#define C_RC_CHANNEL_VR        6

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

void c_rc_receiver_init();
void c_rc_calibrateCenters();
float  c_rc_receiver_getChannel(int channel_n);
int  c_rc_receiver_getCenteredChannel(int channel_n);
float32_t c_rc_receiver_getNormalizedChannel(int channel_n);

#ifdef __cplusplus
}
#endif

#endif //C_RC_RECEIVER_H
