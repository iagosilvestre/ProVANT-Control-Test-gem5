/**
  ******************************************************************************
  * @file    modules/datapr/c_datapr_filter.c
  * @author  Rodrigo Donadel
  * @version V1.0.0
  * @date    06-August-2014
  * @brief   Filtros discretos, a principio para a filtragem de sinais dos sensores.
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_datapr_filter.h"

/** @addtogroup Module_IO
  * @{
  */

/** @addtogroup Module_IO_Filter
  *	\brief Filtragem em geral.
  *
  *	Este componente é projetado para implementar filtros e estimadores (exceto o estimador da attitude, que possui seu próprio modulo).
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float estimated_linear_velocity_z=0;
float last_estimated_linear_velocity_z=0;
float estimated_height=0;
float last_inertial_acc_z=0;

// Integrate the accelerometer data two times to obtain the estimated height
 float c_datapr_filter_estimate_height_acc(float *raw_acc, float *attitude){
	 float inertial_acc_z=0;

	 inertial_acc_z = (-raw_acc[0]*sin(attitude[1]) + raw_acc[1]*cos(attitude[1])*sin(attitude[0]) + raw_acc[2]*cos(attitude[1])*cos(attitude[0])-1)*G;

	 // Integrate acceleration in z to obtain estimated linear velocity in Z inertial axis
	 estimated_linear_velocity_z = c_rc_integrate_trapezoidal(estimated_linear_velocity_z, inertial_acc_z, last_inertial_acc_z, 0.005);
	 // Integrate estimated linear velocity in Z inertial axis to obtain estimated UAV height
	 estimated_height = c_rc_integrate_trapezoidal(estimated_height, estimated_linear_velocity_z, last_estimated_linear_velocity_z, 0.005);

	 // Estimator's memory
	 last_estimated_linear_velocity_z = estimated_linear_velocity_z;
	 last_inertial_acc_z = inertial_acc_z;


	 return estimated_height;
 }

 void reset_height_estimation(float new_initial_height, float new_initial_velocity_z){
	 estimated_linear_velocity_z=new_initial_velocity_z;
	 last_estimated_linear_velocity_z=0;
	 estimated_height=new_initial_height;
 }

