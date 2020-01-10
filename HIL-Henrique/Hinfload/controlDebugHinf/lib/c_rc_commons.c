/**
  ******************************************************************************
  * @file    modules/rc/c_rc_commons.c
  * @author  Rodrigo Donadel
  * @version V1.0.0
  * @date    10-December-2014
  * @brief   Defines e funcoes comuns para os diferentes controladores.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_rc_commons.h"

void c_rc_commons_init(bool manual_height_control){
	c_rc_commons_manual_height_control = manual_height_control;
}

/** \brief Integral numérica utilizando o método trapezoidal (Tustin) */
float c_rc_integrate_trapezoidal(float last_integration, float current_value, float last_value, float sample_time){

	return last_integration + sample_time * 0.5* (current_value + last_value);
}

float c_rc_saturation(float value, float lower_limit, float upper_limit){

	if (value <= lower_limit)
		return lower_limit;
	else if (value >= upper_limit)
		return upper_limit;
	else
		return value;
}

void set_manual_height_control(bool manual_height_control){
	c_rc_commons_manual_height_control = manual_height_control;
}
bool get_manual_height_control(){
	return c_rc_commons_manual_height_control;
}
