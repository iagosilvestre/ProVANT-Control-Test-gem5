/**
  ******************************************************************************
  * @file    modules/rc/pv_module_rc.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   ...
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "pv_module_co.h"
#include "c_control_lqrArthur_vel.h"
//#include "c_control_lqrArthur.h"
#include "pv_typedefs.h"

/** @addtogroup ProVANT_app
  * @{
  */

/** @addtogroup app_co
  * \brief Módulo com as principais funcionalidades para calculo de controle e escrita de atuadores.
  *
  * Definição do módulo.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MODULE_PERIOD	 12//ms
#define ESC_ON           1
#define SERVO_ON         1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
pv_msg_input iInputData;
pv_msg_controlOutput oControlOutputData; 
pv_type_datapr_position iRefData;
pv_type_datapr_position pos_ref;
GPIOPin LED5;

/* Inboxes buffers */
pv_type_actuation    iActuation;
/* Outboxes buffers*/

/* Private function prototypes -----------------------------------------------*/
unsigned char setPointESC_Forca(float forca);
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao do módulo de controle + output.
  *
  * Instancia as Queues de comunicação inter-thread, inicializa a pinagem necessária para
  * os perifericos e aloca o que for necessário para as equações de controle.
  * @param  None
  * @retval None
  */
void module_co_init() 
{

  /* Inicializar os servos */
  //feito no module in

  /*Inicializar o tipo de controlador*/


  /* Pin for debug */
  LED5 = c_common_gpio_init(GPIOD, GPIO_Pin_14, GPIO_Mode_OUT); //LD5

  pv_interface_co.iInputData          = xQueueCreate(1, sizeof(pv_msg_input));
  pv_interface_co.oControlOutputData  = xQueueCreate(1, sizeof(pv_msg_controlOutput));
  pv_interface_co.iRefData  = xQueueCreate(1, sizeof(pv_type_datapr_attitude));
  //c_control_lqrArthur_init();
  c_control_lqrArthur_vel_init();
  pos_ref.x = 2.0;
  pos_ref.y = 0.0;
  pos_ref.z = 1.5;
}

/** \brief Função principal do módulo de RC.
  * @param  None
  * @retval None
  *
  * Interpreta o recebimento de PPM, calcula sinais de controle e os envia
  * via interface.
  * Devido as diferenças do modelo matematica com a construção mecanica o sinal do angulo do servo direito deve 
  * ser adaptado.
  *
  */
void module_co_run() 
{
	unsigned int heartBeat=0;


	/* Inicializa os dados da attitude*/
	oControlOutputData.actuation.servoRight = 0;
	oControlOutputData.actuation.servoLeft  = 0;
	oControlOutputData.actuation.escRightNewtons = 10.2751;
	oControlOutputData.actuation.escLeftNewtons = 10.2799;

  while(1) 
  {
	/* Variavel para debug */
	heartBeat+=1;

	/* Leitura do numero de ciclos atuais */
	lastWakeTime = xTaskGetTickCount();

	if (xQueueReceive(pv_interface_co.iRefData, &iRefData, 0) == pdTRUE)
	{
		/*pos_ref.x += iRefData.x;
		pos_ref.y += iRefData.y;
		pos_ref.z += iRefData.z;*/
		pos_ref.x = iRefData.x;
		pos_ref.y = iRefData.y;
		pos_ref.z = iRefData.z;
		pos_ref.dotX = iRefData.dotX;
		pos_ref.dotY = iRefData.dotY;
		pos_ref.dotZ = iRefData.dotZ;

	}


	/* Passa os valores davariavel compartilha para a variavel iInputData */
	if (xQueueReceive(pv_interface_co.iInputData, &iInputData, 0) == pdTRUE)
	{
		iInputData.position_reference.x = pos_ref.x;
		iInputData.position_reference.y = pos_ref.y;
		iInputData.position_reference.z = pos_ref.z;
		iInputData.position_reference.dotX = pos_ref.dotX;
		iInputData.position_reference.dotY = pos_ref.dotY;
		iInputData.position_reference.dotZ = pos_ref.dotZ;
		oControlOutputData.actuation = c_control_lqrArthur_vel_controller(iInputData);
		//oControlOutputData.actuation = c_control_lqrArthur_controller(iInputData);
	}

	/* toggle pin for debug */
	c_common_gpio_toggle(LED5);

	if(pv_interface_co.oControlOutputData != 0)
	  xQueueOverwrite(pv_interface_co.oControlOutputData, &oControlOutputData);

	/* A thread dorme ate o tempo final ser atingido */
	vTaskDelayUntil( &lastWakeTime, MODULE_PERIOD / portTICK_RATE_MS);
	}
}

/* #<{(|*\ brief Calcula o set point do ESC a partir da forca passada por argumento */
/*  * Curva retirada dos ensaios com os motores brushless no INEP */
/*  |)}># */
/* unsigned char setPointESC_Forca(float forca){ */
/* 	//	Coefficients: */
/* 	float p1 = 0.00088809, p2 = -0.039541, p3 = 0.67084, p4 = -5.2113, p5 = 16.33, p6 = 10.854, p7 = 3.0802, set_point=0; */
/*  */
/* 	if (forca <= 0) */
/* 		return (unsigned char) ESC_MINIMUM_VELOCITY; */
/* 	else{ */
/* 		set_point = (p1*pow(forca,6) + p2*pow(forca,5) + p3*pow(forca,4) + p4*pow(forca,3) */
/* 								+ p5*pow(forca,2) + p6*forca + p7); */
/* 	    if (set_point >= 255) */
/* 	    	return (unsigned char)255; */
/* 	    else */
/* 	    	return (unsigned char)set_point;} */
/* } */
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */
