/**
  ******************************************************************************
  * @file    modules/datapr/c_datapr_MahonyAHRS.c
  * @author  Rodrigo Donadel
  * @version V1.0.0
  * @date    8-Jul-2014
  * @brief   Madgwick's implementation of Mayhony's AHRS algorithm.
  *          See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
  *****************************************************************************/

/** 
	 Date     | Author       | Notes
	----------|--------------|----------------
	29/09/2011| SOH Madgwick | Initial release
	02/10/2011| SOH Madgwick | Optimised for reduced CPU load
***********************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef c_datapr_MahonyAHRS_h
#define c_datapr_MahonyAHRS_h

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
//extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

/* Exported functions ------------------------------------------------------- */

void c_datapr_MahonyAHRSupdate(float * q, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, long sample_time_gyro_us);
void c_datapr_MahonyAHRSupdateIMU(float * q, float gx, float gy, float gz, float ax, float ay, float az);


#ifdef __cplusplus
}
#endif

#endif
