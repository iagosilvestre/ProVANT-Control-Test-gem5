/**
  ******************************************************************************
  * @file    modules/control/c_control_HinfLoad.c
  * @author  Arthur Viana Lara
  * @version V1.0.0
  * @date    28-Novembro-2017
  * @brief   Controle de rastreamento de trajetória por realimentacao de estados 
  *	     calculado com a estratégia de controle misto Hinfinito/H2.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/

#include "c_common_uart.h"
#include "c_io_protocolP2P.h"
#include "c_common_utils.h"
#include "pv_typedefs.h"

/** @addtogroup Module_RC
  * @{
  */

/** @addtogroup Module_control_Component_State_Feedback_Control
  * \brief Controle de estabilização para o modo de operação RC por realimentacao de estados.
  *
   * @{
  */

		//---------------------------------------------------------------------------------------------

/* Exported functions definitions --------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/




pv_type_actuation actuation_signals;


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/




/* Exported functions definitions --------------------------------------------*/

void c_control_HinfLoad_init()
{
	// inicializa matrizes estáticas
	
	
}

pv_type_actuation c_control_HinfLoad_controller(pv_msg_input input)
{

	float32_t Hinfload_K_f32_hil[4][20]=
{{-0.000509474023994 ,  1.381002006541810 ,  2.044930990723325 , -4.098388643419657 ,  0.002544968427177 ,  0.065243786421189, -0.011997162152724 ,  0.012231188237446 , -0.000191109567030 ,  0.977046060078436  , 2.067519443836474 , -1.069820095142832,
0.003981280957280 ,  0.048837474399233 , -0.000157117358773  , 0.000160465562387 , -0.000324184653490 ,  0.932396154093819,
0.987708499654750 ,  0.029590812630684},

{0.000485844262540 , -1.379777079391250 ,  2.046778453514912 ,  4.095526259295323 ,  0.006483505671600 , -0.065184427882726,
0.011988462007403 , -0.012218371367099 , 0.000647666101473 , -0.976219155907380 ,  2.069372953111319  , 1.069647999496566,
0.005073107886213 , -0.048792319825525 ,  0.000156410506717 , -0.000160891309965 ,  0.000193352263628 , -0.931559941080183,
0.988594878555003 , -0.029564069576714},

{0.147511637570957 , -0.044700625197703  , 0.000027359219549  , 0.100038920576059  , 0.186220549413359 ,  0.044874312445416,
0.218427852703492 ,  0.036876304589717 ,  0.070402232740872 , -0.028416542202667 ,  0.000007473706141 ,  0.017792742949581,
0.025919243351085 ,  0.030976534992245 ,  0.007155474366137 ,  0.000723596619702 ,  0.140517760855087 , -0.032497295109449,
0.000009086030883 ,  0.021343249034188},

{0.147480373299421 ,  0.044731365984124 ,  0.000026937597506 , -0.099883806663005 ,  0.186155931716280 , -0.044878136021767,
0.036878957916608 ,  0.218443336829840 ,  0.070383903601564 ,  0.028402976825487  , 0.000007312089313 , -0.017740572220967,
0.025908596148404,  -0.030981560936545  , 0.000723658016145  , 0.007155720063258  , 0.140489293081162 ,  0.032562443693621,
0.000009316184096 , -0.021344301330953}
};

	float32_t error_state_vector_f32_hil[20]={0};
	float32_t control_output_f32_hil[4]={0};
	float32_t state_vector_f32_hil[20]={0};
	float32_t reference_f32_hil[20]={0};


	arm_matrix_instance_f32 Hinfload_K_hil;
	arm_matrix_instance_f32 error_state_vector_hil;
	arm_matrix_instance_f32 control_output_hil;
	arm_matrix_instance_f32 state_vector_hil;
	arm_matrix_instance_f32 reference_hil;	


	// mémoria para integradores
	static float32_t xint = 0;
	static float32_t x_ant = 0;
        static float32_t yint= 0;
	static float32_t y_ant = 0;
        static float32_t zint= 0;
	static float32_t z_ant = 0;
        static float32_t yawint = 0;
	static float32_t yaw_ant = 0;

	float32_t x_atual = 0;
	float32_t y_atual = 0;
	float32_t z_atual = 0;
	float32_t yaw_atual = 0;

	// criando vetor de refência
	reference_f32_hil[0]= 2;
	reference_f32_hil[1]= 0;
	reference_f32_hil[2]= 1.5;
	reference_f32_hil[3]= 0;
	reference_f32_hil[4]= 0;
	reference_f32_hil[5]= 0;
	reference_f32_hil[6]= 0;
	reference_f32_hil[7]= 0;
	reference_f32_hil[8]= 0;
	reference_f32_hil[9]= 0;
	reference_f32_hil[10]= 0;
	reference_f32_hil[11]= 0;
	reference_f32_hil[12]= 0;
	reference_f32_hil[13]= 0;
	reference_f32_hil[14]= 0;
	reference_f32_hil[15]= 0;
	reference_f32_hil[16]= 0;
	reference_f32_hil[17]= 0;
	reference_f32_hil[18]= 0;
	reference_f32_hil[19]= 0;

	//Frame teste = frame_create();
	//for(int i=0; i<24;i++) frame_addFloat(&teste, reference_f32_hil[i]);
	//frame_build(&teste);
	//c_io_protocolP2P_send(&teste);
	
	// Integrador Trapezoidal
	x_atual = input.position.x - input.position_reference.x;
        xint = xint + (0.012/2.0)*(x_atual + x_ant);
        x_ant = x_atual;
        y_atual = input.position.y - input.position_reference.y;
        yint = yint + (0.012/2.0)*(y_atual + y_ant);
        y_ant = y_atual;
        z_atual = input.position.z - input.position_reference.z;
        zint = zint + (0.012/2.0)*(z_atual + z_ant);
        z_ant = z_atual;
        yaw_atual = input.attitude.yaw;
        yawint = yawint + (0.012/2.0)*(yaw_atual + yaw_ant);
        yaw_ant = yaw_atual;
	
	// criando vetor de estado
	
	state_vector_f32_hil[0]= input.position.x;
	state_vector_f32_hil[1]= input.position.y;
	state_vector_f32_hil[2]= input.position.z;
	state_vector_f32_hil[3]= input.attitude.roll;
	state_vector_f32_hil[4]= input.attitude.pitch;
	state_vector_f32_hil[5]= input.attitude.yaw;
	state_vector_f32_hil[7]= input.servosOutput.servo.alphar;
	state_vector_f32_hil[8]= input.servosOutput.servo.alphal;
	state_vector_f32_hil[9]= input.position.dotX;
	state_vector_f32_hil[10]= input.position.dotY;
	state_vector_f32_hil[11]= input.position.dotZ;
	state_vector_f32_hil[12]= input.attitude.dotRoll;/* + 
			      input.attitude.dotYaw*cos(input.attitude.roll)*tan(input.attitude.pitch) + 
			      input.attitude.dotPitch*sin(input.attitude.roll)*tan(input.attitude.pitch); */
	state_vector_f32_hil[13]= input.attitude.dotPitch;/**cos(input.attitude.roll) - 
			      input.attitude.dotYaw*sin(input.attitude.roll); */
	state_vector_f32_hil[14]= input.attitude.dotYaw;/* *cos(input.attitude.roll))/cos(input.attitude.pitch) + 
			      (input.attitude.dotPitch*sin(input.attitude.roll))/cos(input.attitude.pitch); */
	state_vector_f32_hil[15]= input.servosOutput.servo.dotAlphar;


				//	c_common_usart_puts(USART6, "\ndAR: ");
				//	char _buff[10];
				//	c_common_utils_floatToString(state_vector_f32_hil[15], _buff, 0);
				//	c_common_usart_puts(USART6, _buff);
				//	state_vector_f32_hil[16]= input.servosOutput.servo.dotAlphal;
				//	c_common_usart_puts(USART6, "\ndAL: ");
				//	char _buff2[10];
				//	c_common_utils_floatToString(state_vector_f32_hil[16], _buff2, 0);
				//	c_common_usart_puts(USART6, _buff2);

	//state_vector_f32_hil[17]= xint;
	//state_vector_f32_hil[18]= yint;
	//state_vector_f32_hil[19]= zint;
	//state_vector_f32_hil[20]= yawint;
	state_vector_f32_hil[17]= 0;
	state_vector_f32_hil[18]= 0;
	state_vector_f32_hil[19]= 0;
	state_vector_f32_hil[20]= 0;

	//Frame teste2 = frame_create();
	//for(int i=0; i<24;i++) frame_addFloat(&teste2, state_vector_f32_hil[i]);
	//frame_build(&teste2);
	//c_io_protocolP2P_send(&teste2);

	arm_mat_init_f32(&Hinfload_K_hil, 4,20,(float32_t *)Hinfload_K_f32_hil);
	arm_mat_init_f32(&error_state_vector_hil, 20,1,(float32_t *)error_state_vector_f32_hil);
	arm_mat_init_f32(&control_output_hil,4,1,(float_t *)control_output_f32_hil);
	arm_mat_init_f32(&state_vector_hil,20,1,(float_t *)state_vector_f32_hil);
	arm_mat_init_f32(&reference_hil,20,1,(float_t *)reference_f32_hil);	

	// lei de controle
	arm_mat_sub_f32(&state_vector_hil, &reference_hil, &error_state_vector_hil);


	arm_mat_mult_f32(&Hinfload_K_hil, &error_state_vector_hil, &control_output_hil);
	
	float32_t input1 = control_output_hil.pData[0];
	float32_t input2 = control_output_hil.pData[1];
	float32_t input3 = control_output_hil.pData[2];
	float32_t input4 = control_output_hil.pData[3];
	
	control_output_hil.pData[0] = (-input1) + 10.2751;
	control_output_hil.pData[1] = (-input2) + 10.2799;
	control_output_hil.pData[2] = (-input3) + 0;
	control_output_hil.pData[3] = (-input4) + 0;
	
					//	c_common_usart_puts(USART6, "\nFr: ");
					//	char buff[10];
					//	c_common_utils_floatToString(control_output_hil.pData[0], buff, 0);
					//	c_common_usart_puts(USART6, buff);
					//	char buff1[10];
					//	c_common_usart_puts(USART6, "\nFl: ");
					//	c_common_utils_floatToString(control_output_hil.pData[1], buff1, 0);
					//	c_common_usart_puts(USART6, buff1);
					//	char buff2[10];
					//	c_common_usart_puts(USART6, "\nTr: ");
					//	c_common_utils_floatToString(control_output_hil.pData[2], buff2, 0);
					//	c_common_usart_puts(USART6, buff2);
					//	char buff3[10];
					//	c_common_usart_puts(USART6, "\nTl: ");
					//	c_common_utils_floatToString(control_output_hil.pData[3], buff3, 0);
					//	c_common_usart_puts(USART6, buff3);

	actuation_signals.escRightNewtons=(float32_t)control_output_hil.pData[0];
	actuation_signals.escLeftNewtons=(float32_t)control_output_hil.pData[1];
	actuation_signals.servoRight=(float32_t)control_output_hil.pData[2];
	actuation_signals.servoLeft=(float32_t)control_output_hil.pData[3];

	return actuation_signals;


}

/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

