/**
  ******************************************************************************
  * @file    modules/control/c_control_LQRIuro.c
  * @author  Iuro Nascimento
  * @version V1.0.0
  * @date    28-July-2016
  * @brief   Implementa controlador LQRIuro baseado em c_rc_LQRIuro_control de Rodrigo Donadel.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_control_LQRIuro.h"

/** @addtogroup Module_Control
  * @{
  */

/** @addtogroup Module_Control_Component_LQRIuro
  * \brief Controlador LQRIuro.
  *
   * @{
  */

		//---------------------------------------------------------------------------------------------

/* Exported functions definitions --------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static pv_type_stability_error last_error={0};

/* static float32_t K_f32[4][8] = {25.1182142186511,	-22.9115803553020,	0.146184396518071,	0.552710269633766,	9.06800934753831,	-2.38866481704336,	0.164187308167002,	0.443301274186907, */
/* 25.0949087679728,	22.5540012159260,	-0.913271949214306,	-0.647392092157934,	9.07708061130279,	2.36194579304949,	0.112430839438279,	-0.532494091057842, */
/* -0.563021822051008,	0.844693654348630,	15.2309559728244,	1.01982186363717,	-0.211676024359927,	0.0702980064638425,	0.774620310054486,	0.809934322059327, */
/* -0.487348615658231,	-1.13713238968602,	8.99075079413709,	-1.69356614922977,	-0.162835094809224,	-0.0943024204994003,	0.500274678117326,	-1.35059403822751}; */

/* K = K/10 */
/* static float32_t K_f32[4][8] = {2.51182142186511.	-2.29115803553021.	0.0146184396518071.	0.0552710269633766.	0.906800934753831.	-0.238866481704336.	0.0164187308167002.	0.0443301274186907. 2.50949087679728.	2.25540012159260.	-0.0913271949214306.	-0.0647392092157934.	0.907708061130279.	0.236194579304949.	0.0112430839438279.	-0.0532494091057842. -0.0563021822051008.	0.0844693654348630.	1.52309559728244.	0.101982186363717.	-0.0211676024359927.	0.00702980064638425.	0.0774620310054486.	0.0809934322059327. -0.0487348615658231.	-0.113713238968602.	0.899075079413709.	-0.169356614922977.	-0.0162835094809224.	-0.00943024204994003.	0.0500274678117326. -0.135059403822751}; */
/* K = K/2 */
/* static float32_t K_f32[4][8] = {12.5591071093256,-11.4557901776510,0.0730921982590353,0.276355134816883,4.53400467376915,-1.19433240852168,0.0820936540835010,0.221650637093454, 12.5474543839864,11.2770006079630,-0.456635974607153,-0.323696046078967,4.53854030565139,1.18097289652474,0.0562154197191395,-0.266247045528921, -0.281510911025504,0.422346827174315,7.61547798641220,0.509910931818585,-0.105838012179964,0.0351490032319212,0.387310155027243,0.404967161029664,-0.243674307829115,-0.568566194843008,4.49537539706854,-0.846783074614883,-0.0814175474046118,-0.0471512102497001,0.250137339058663,-0.675297019113757}; */

/* DLQRIuro */
static float32_t K_f32[4][8] = {9.76366801344208,-7.51300560059946,-0.428328868054615,0.221182931643446,4.90384308706740,-1.11741465945060,0.0779241229453429,0.180398726655569,9.72887283685950,7.23920114755071,-0.657535697917373,-0.259883732993132,4.90090261046043,1.08809322748406,0.0623056079414057,-0.224024791393927,-0.400359536206610,0.213190289794702,4.17878050466461,0.184031484341207,-0.216800126639690,0.0215640263786921,0.282375003574583,0.147600371756559,-0.346318335424635,-0.381361309842456,3.02618191496790,-0.487136023446626,-0.174790942683399,-0.0376480965949778,0.226729327308100,-0.394086702196973};

static float32_t equilibrium_point_f32[8]={1,0.000203636,0.0897526,0.0,0.0,0.0,0.0,0.0};
static float32_t equilibrium_control_f32[4]={9.94932000000000,	9.98525000000000, 0.115415000000000, 0.114677000000000};
static float32_t state_vector_f32[8]={0};
static float32_t error_state_vector_f32[8]={0};
static float32_t control_output_f32[4]={0};
static float32_t delta_control_f32[4]={0};

static arm_matrix_instance_f32 equilibrium_control;
static arm_matrix_instance_f32 K;


/* Private function prototypes -----------------------------------------------*/
static arm_matrix_instance_f32 c_control_LQRIuro_calcErrorStateVector(pv_type_datapr_attitude attitude, pv_type_datapr_attitude attitude_reference, pv_type_datapr_position position, pv_type_datapr_position position_reference);

/* Private functions ---------------------------------------------------------*/
arm_matrix_instance_f32 c_control_LQRIuro_calcErrorStateVector(pv_type_datapr_attitude attitude, pv_type_datapr_attitude attitude_reference,
		pv_type_datapr_position position, pv_type_datapr_position position_reference) {

	arm_matrix_instance_f32 error_state_vector, state_vector, equilibrium_point;

	//State Vector
	state_vector_f32[STATE_Z]=position.z;
	state_vector_f32[STATE_ROLL]=attitude.roll;
	state_vector_f32[STATE_PITCH]=attitude.pitch;
	state_vector_f32[STATE_YAW]=attitude.yaw;
	state_vector_f32[STATE_DZ]=position.dotZ;
	state_vector_f32[STATE_DROLL]=attitude.dotRoll;
	state_vector_f32[STATE_DPITCH]=attitude.dotPitch;
	state_vector_f32[STATE_DYAW]=attitude.dotYaw;

	//Updates the height equilibrium point according to the reference
	equilibrium_point_f32[STATE_Z]= position_reference.z;
	/* equilibrium_point_f32[STATE_DZ]= position_reference.dotZ; */
	equilibrium_point_f32[STATE_ROLL]= attitude_reference.roll;
	equilibrium_point_f32[STATE_PITCH]= attitude_reference.pitch;

	//Initializes the matrices
	arm_mat_init_f32(&equilibrium_point, 8, 1, (float32_t *)equilibrium_point_f32);
	arm_mat_init_f32(&state_vector, 8, 1, (float32_t *)state_vector_f32);
	arm_mat_init_f32(&error_state_vector, 8, 1, (float32_t *)error_state_vector_f32);

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
void c_control_LQRIuro_init() {

	// Inicializa as matrizes estaticas
	arm_mat_init_f32(&equilibrium_control, 4, 1, (float32_t *)equilibrium_control_f32);
	arm_mat_init_f32(&K, 4, 8, (float32_t *)K_f32);
}



/** \brief LQRIuro Controller.  */
pv_type_actuation c_control_LQRIuro_controller(pv_type_datapr_attitude attitude,
				  pv_type_datapr_attitude attitude_reference,
				  pv_type_datapr_position position,
				  pv_type_datapr_position position_reference,
				  bool manual_height_control){

	pv_type_actuation actuation_signals;
	arm_matrix_instance_f32 error_state_vector, delta_control, control_output;
	
	//Initialize result matrices
	arm_mat_init_f32(&control_output, 4, 1, (float32_t *)control_output_f32);
	arm_mat_init_f32(&delta_control,4,1,(float32_t *)delta_control_f32);
	pv_type_stability_error error;
	float temp_height_takeoff=0;

	error_state_vector = c_control_LQRIuro_calcErrorStateVector(attitude, attitude_reference, position, position_reference);

	/* -delta_u = K*delta_x */
	arm_mat_mult_f32(&K, &error_state_vector, &delta_control);
	/* u = ur - delta_u */
	arm_mat_sub_f32(&equilibrium_control, &delta_control, &control_output);
	//
	//The result must be in a struct pv_msg_io_actuation
	actuation_signals.escRightSpeed= (float)control_output.pData[0];
	actuation_signals.escLeftSpeed=	 (float)control_output.pData[1];
	actuation_signals.servoRight=	 (float)control_output.pData[2];
	actuation_signals.servoLeft=	 (float)control_output.pData[3];
    //Declares that the servos will use angle control, rather than torque control
	actuation_signals.servoTorqueControlEnable = 0;

	return actuation_signals;
}

/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

