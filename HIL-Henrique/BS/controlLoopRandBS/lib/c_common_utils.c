/**
  ******************************************************************************
  * @file    modules/common/c_common_utils.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    10-February-2014
  * @brief   Funções gerais para utilização em outros módulos.
  *
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_common_utils.h"

/** @addtogroup Common_Components
  * @{
  */

/** @addtogroup Common_Components_Utils
  *
  * \brief Implementa funções utilitárias para o projeto.
  *
  * Aqui são adicionadas funções genéricas (matemáticas, etc.) passíveis de utilização
  * em todos os módulos.
  *
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define CORE_SysTickEn()    (*((u32*)0xE0001000)) = 0x40000001 /** Inicia SysTick - não usar com FreeRTOS rodando! */
#define CORE_SysTickDis()   (*((u32*)0xE0001000)) = 0x40000000 /** Pára SysTick - não usar com FreeRTOS rodando! */
#define CORE_GetSysTick()   (*((u32*)0xE0001004)) /** Retorna valor atua do registrador de Systick */

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/**
 * @brief Função para reiniciar o software
 * @details Esta função chama outra função da biblioteca CMSIS para realizar o reset da placa, fazendo desta forma que todos os valores
 * de variaveis sejam modificadas para o valor original do programa, assim sendo um "hard-reset".
 *
 */
void c_common_utils_resetSystem()
{
	NVIC_SystemReset();
}


/** \brief Função de mapeamento de variável de um intervalo a outro.
 *
 * Mapeia a variável \f$ x \f$ de um intervalo \f$ [in_{min} , in_{max}] \f$ para \f$ [out_{min} , out_{max}] \f$.
 */
float c_common_utils_map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/** \brief Satura a variável x entre [min, max].
 *
 * Para \f$ x \in [min, max], x = x \f$. Para \f$ x > max, x = max \f$; para \f$ x < min, x = min \f$.
 *
 * @retval x após saturação.
 */
float c_common_utils_sat(float x, float min, float max) {
	if (x > max) return max;
	else if (x < min) return min;
	else return x;
}

/** \brief Delay de us baseado em SysTick.
 *
 * Ainda implementado como busy waiting! Alterar...
 */
void  c_common_utils_delayus(long us) {
	CORE_SysTickEn();
	us = us*(SystemCoreClock / 1000000);
	vu32 t1,t2;
	t1 = CORE_GetSysTick();
	do { t2 = CORE_GetSysTick(); } while((t2-t1) < us);
}

/** \brief Delay de ms baseado em SysTick.
 *
 * Ainda implementado como busy waiting! Alterar...
 */
void  c_common_utils_delayms(int ms) {
	CORE_SysTickEn();
	ms = ms*(SystemCoreClock / 1000);
	vu32 t1,t2;
	t1 = CORE_GetSysTick();
	do { t2 = CORE_GetSysTick(); } while((t2-t1) < ms);
}

/** \brief Formata um float para impressão via printf, UART, etc.
 *
 * Complementa o sprintf - que não é identico ao PC no ambiente embedded.
 *
 * @param num Número float a ser formatado.
 * @param *outbuf Buffer no qual o resultado será armazenado.
 * @param decplaces Quantidade de casas após a vírgula a serem mostrada (não implementado).
 */
void c_common_utils_floatToString(float num, char * outbuf, char decplaces) {
	int bComma, aComma; //before and after Comma
	float subnum;
	bComma = (int) num; 	//truncates number automatically
	subnum = num - bComma; 	//leaves only digits after the comma
	aComma = ((int)(subnum*10000.0f));

	if(num > 0)
		sprintf(outbuf, "%d.%04d", abs(bComma), abs(aComma));
	else
		sprintf(outbuf, "-%d.%04d", abs(bComma), abs(aComma));
}

/** \brief Retorna o valor do registrador de Systicks desde o disparo do contador.
 *
 */
long c_common_utils_getSysTickCount() {
	return (long) CORE_GetSysTick();
}

/** \brief Habilita o SysTick.
 *
 * É compatível com o FreeRTOS e pode ser chamado mesmo com o SysTick já rodando, sem alterar seu valor.
 */
void c_common_utils_enSysTick() {
	CORE_SysTickEn();
}

long c_common_utils_micros() {
	CORE_SysTickEn();
	return (long)(CORE_GetSysTick()/(SystemCoreClock/1000000));
}

/** \brief Retorna o valor em milissegundos desde o disparo do SysTick.
 *
 * Caso o Registrador de SysTick não esteja rodando quando a função é chamada, o registrador é ativado
 * via c_common_utils_enSysTick() .
 */
long c_common_utils_millis() {
	CORE_SysTickEn();
	return (long)(CORE_GetSysTick()/(SystemCoreClock/1000));
}



/**
  * @}
  */

/**
  * @}
  */

