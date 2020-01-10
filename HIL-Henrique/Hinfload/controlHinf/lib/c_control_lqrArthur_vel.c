/**
  ******************************************************************************
  * @file    modules/control/c_control_lqrArthur.c
  * @author  Iuro Nascimento
  * @version V1.0.0
  * @date    28-July-2016
  * @brief   Implementa controlador lqrArthur baseado em c_rc_lqrArthur_control de Rodrigo Donadel.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_control_lqrArthur_vel.h"


//---------------------------------------------------------------------------------------------

/* Exported functions definitions --------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static pv_type_stability_error last_error={0};

/* DlqrArthur */

static float32_t K_f32[4][17] ={{-2.919664,0.003449,0.044031,-0.008393,0.008407,0.000104,0.747741,1.402244,-0.669800,0.004615,0.031676,-0.000087,0.000087,0.020409,0.000006,0.938887,0.989747},
		{2.910650,0.003197,-0.043875,0.008376,-0.008364,-0.000011,-0.745117,1.407336,0.669261,0.004589,-0.031563,0.000087,-0.000087,-0.020337,-0.000274,-0.935521,0.993303},
		{0.044896,0.160841,0.038164,0.229241,0.021664,0.063339,-0.013700,0.000038,0.007160,0.020867,0.025705,0.006099,0.000353,0.018364,0.122018,-0.019185,0.000016},
		{-0.044764,0.161088,-0.038120,0.021688,0.229734,0.063410,0.013658,0.000036,-0.007140,0.020900,-0.025689,0.000354,0.006104,-0.018339,0.122032,0.019123,0.000016}};


/*static float32_t K_f32[4][20] = {{-0.000509474023994 ,  1.381002006541810 ,  2.044930990723325 , -4.098388643419657 ,  0.002544968427177 ,  0.065243786421189, -0.011997162152724 ,  0.012231188237446 , -0.000191109567030 ,  0.977046060078436  , 2.067519443836474 , -1.069820095142832,
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
};*/

static float32_t equilibrium_point_f32[17]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
static float32_t equilibrium_control_f32[4]={10.2751,	10.2799, 0, 0};
static float32_t state_vector_f32[17]={0};
static float32_t error_state_vector_f32[17]={0};
static float32_t control_output_f32[17]={0};
static float32_t delta_control_f32[17]={0};

static arm_matrix_instance_f32 equilibrium_control;
static arm_matrix_instance_f32 K;


/* Private function prototypes -----------------------------------------------*/
/* static arm_matrix_instance_f32 c_control_lqrArthur_calcErrorStateVector(pv_type_datapr_attitude attitude, pv_type_datapr_attitude attitude_reference, pv_type_datapr_position position, pv_type_datapr_position position_reference); */
arm_matrix_instance_f32 c_control_lqrArthur_vel_calcErrorStateVector(pv_msg_input inputData);

/* Private functions ---------------------------------------------------------*/
arm_matrix_instance_f32 c_control_lqrArthur_vel_calcErrorStateVector(pv_msg_input inputData) {
	pv_type_datapr_attitude attitude = inputData.attitude;
	pv_type_datapr_attitude attitude_reference = inputData.attitude_reference;
	pv_type_datapr_position position = inputData.position;
	pv_type_datapr_position position_reference = inputData.position_reference;
	pv_type_datapr_servos servos = inputData.servosOutput.servo;
	arm_matrix_instance_f32 error_state_vector, state_vector, equilibrium_point;


	static float32_t xint = 0, x_ant = 0;
	static float32_t yint = 0, y_ant = 0;
	static float32_t zint = 0, z_ant = 0;
	static float32_t yawint = 0, yaw_ant = 0;
	
	float32_t T = 0.012;

	// Integrador Trapezoidal
	double x_atual = position.dotX - position_reference.dotX;
	xint = xint + (T/2)*(x_atual + x_ant);
	x_ant = x_atual;
	double y_atual = position.dotY - position_reference.dotY;
	yint = yint + (T/2)*(y_atual + y_ant);
	y_ant = y_atual;
	double z_atual = position.dotZ - position_reference.dotZ;
	zint = zint + (T/2)*(z_atual + z_ant);
	z_ant = z_atual;
	double yaw_atual = attitude.yaw;
	yawint = yawint + (T/2)*(yaw_atual + yaw_ant);
	yaw_ant = yaw_atual;

	//State Vector
	//state_vector_f32[0]=position.x;
	//state_vector_f32[1]=position.y;
	//state_vector_f32[2]=position.z;
	state_vector_f32[0]=attitude.roll;
	state_vector_f32[1]=attitude.pitch;
	state_vector_f32[2]=attitude.yaw;
	state_vector_f32[3]=servos.alphar;
	state_vector_f32[4]=servos.alphal;
	state_vector_f32[5]=position.dotX;
	state_vector_f32[6]=position.dotY;
	state_vector_f32[7]=position.dotZ;
	state_vector_f32[8]=attitude.dotRoll;
	state_vector_f32[9]=attitude.dotPitch;
	state_vector_f32[10]=attitude.dotYaw;
	state_vector_f32[11]=servos.dotAlphar;
	state_vector_f32[12]=servos.dotAlphal;
	state_vector_f32[13]=yawint;
	state_vector_f32[14]=xint;
	state_vector_f32[15]=yint;
	state_vector_f32[16]=zint;

	//Updates the height equilibrium point according to the reference
	/*equilibrium_point_f32[0]= position_reference.x;
	/equilibrium_point_f32[1]= position_reference.y;
	equilibrium_point_f32[2]= position_reference.z;
	equilibrium_point_f32[5]= attitude_reference.yaw; */


	//Initializes the matrices
	arm_mat_init_f32(&equilibrium_point, 17, 1, (float32_t *)equilibrium_point_f32);
	arm_mat_init_f32(&state_vector, 17, 1, (float32_t *)state_vector_f32);
	arm_mat_init_f32(&error_state_vector, 17, 1, (float32_t *)error_state_vector_f32);

	//e(t)=x(t)- equilibrium_point
	arm_mat_sub_f32(&state_vector, &equilibrium_point, &error_state_vector);

	return error_state_vector;
}


/* Exported functions definitions --------------------------------------------*/

/** \brief Inicilização do controle de estabilidade.
 *
 * O controlador utiliza a API de DSP da CMSIS, e portanto se baseia fortemente no uso do
 * tipo arm_matrix_instance_f32. Esta \b struct contêm os valores de número de linhas e
 * colunas de matriz, além de um ponteiro para seus elementos (na forma de array).
 * Estes arrays são prealocados globalmente (ver código fonte), para evitar overhead
 * de alocação dinâmica em cada chamada e para evitar que, a cada alocação em uma função, a memória para
 * a qual o ponteiro aponta saia de escopo e seja deletada. Uma vez que as funções são privadas e chamadas
 * em ordem determinística, mutexes não são implementadas (por simplicidade apenas)
 */
void c_control_lqrArthur_vel_init() {

	// Inicializa as matrizes estaticas
	arm_mat_init_f32(&equilibrium_control, 4, 1, (float32_t *)equilibrium_control_f32);
	arm_mat_init_f32(&K, 4, 17, (float32_t *)K_f32);
}



/** \brief lqrArthur Controller.  */
pv_type_actuation c_control_lqrArthur_vel_controller(pv_msg_input inputData){

	/*pv_type_datapr_attitude attitude = inputData.attitude;
	pv_type_datapr_attitude attitude_reference = inputData.attitude_reference;
	pv_type_datapr_position position = inputData.position;
	pv_type_datapr_position position_reference = inputData.position_reference;
	pv_type_datapr_servos servos = inputData.servosOutput.servo;*/
	pv_type_actuation actuation_signals;
	
	arm_matrix_instance_f32 error_state_vector, delta_control, control_output;
	
	//Initialize result matrices
	arm_mat_init_f32(&control_output, 4, 1, (float32_t *)control_output_f32);
	arm_mat_init_f32(&delta_control,4,1,(float32_t *)delta_control_f32);
	pv_type_stability_error error;
	float temp_height_takeoff=0;

	error_state_vector = c_control_lqrArthur_vel_calcErrorStateVector(inputData);

	/* -delta_u = K*delta_x */
	arm_mat_mult_f32(&K, &error_state_vector, &delta_control);
	/* u = ur - delta_u */
	arm_mat_sub_f32(&equilibrium_control, &delta_control, &control_output);
	//
	//The result must be in a struct pv_msg_io_actuation
	actuation_signals.escRightNewtons= (float)control_output.pData[0];
	actuation_signals.escLeftNewtons=	 (float)control_output.pData[1];
	actuation_signals.servoRight=	 (float)control_output.pData[2];
	actuation_signals.servoLeft=	 (float)control_output.pData[3];
    //Declares that the servos will use angle control, rather than torque control
	actuation_signals.servoTorqueControlEnable = 0;

	return actuation_signals;
}

/* IRQ handlers ------------------------------------------------------------- */


