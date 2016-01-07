//
//		INTRODUCAO A DIGITAL POWER COM A STM32F334
//
//		@file  main.c
//      @brief Essa aplicacao controla o boost digital agora em modo
//			   closed loop. A tensao de saida e ajustada atraves de
//			   PA3.
//



#include "stm32f30x.h"
#include "stm32f3348_discovery.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_dac.h"
#include "stm32f30x_adc.h"
#include "boost.h"

//
// Macros e constantes uteis:
//


#define TICK_FREQ      1000   //frequencia de updade do tick.
#define SYSTICK_LOAD_VAL(x) (x == 0?0:(SystemCoreClock) /x)    //recarga do systick
#define BUTTON_SCAN_TIME 10   //periodo de scanning do botao em ms
#define V_SETPOINT_SCALE  3.662e-3f //fator de conversao do setpoint pra volts
#define ADC_TO_VOLTAGE(x) ((float)x * (V_SETPOINT_SCALE))


#define DIMMING_MAX_VAL  15.0f
#define DIMMING_MIN_VAL  5.50f

//
// Porta usada para ajuste de setpoint:
//
#define DIMMING_SET_POINT_PORT GPIOA
#define DIMMING_SET_POINT_GPIO GPIO_Pin_7


//
// maquininha simples de estado do dimmer:
//
typedef enum
{
	kdimmingUp = 0,
	kdimmingDown,
}dim_state;

//
// Variaveis:
//
uint32_t tickCounter = 0;
dim_state dimmingMchn = kdimmingUp;


//
// referencias de funcoes estaticas:
//
static void 	InitUserAdc(void);
static uint16_t GetUserAdc(void);

//
// @fn InitUserAdc()
// @brief Inicializa ADC do usuario para pegar a conversao de set point
//
void InitUserAdc(void)
{
	GPIO_InitTypeDef analogIo;
	analogIo.GPIO_Mode = GPIO_Mode_AN;
	analogIo.GPIO_Pin = DIMMING_SET_POINT_GPIO;


	//Aciona clock do port onde esta localizado o A/D
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	//Inicializa o pino de IO desejado:
	GPIO_Init(DIMMING_SET_POINT_PORT, &analogIo);

	//Inicializa clock do ADC:
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);

	// Configura o A/D:
	ADC_InitTypeDef adcInit;
	ADC_CommonInitTypeDef adcCommon;

	ADC_StructInit(&adcInit);
	ADC_CommonStructInit(&adcCommon);

	//
	// o adc vai rodar no modo mais simples, single conversion + 1 regular channel:
	//
	adcCommon.ADC_Clock = ADC_Clock_SynClkModeDiv1;
	adcCommon.ADC_Mode = ADC_Mode_Independent;

	adcInit.ADC_ExternalTrigEventEdge = ADC_ExternalTrigInjecEventEdge_None;
	adcInit.ADC_AutoInjMode = ADC_AutoInjec_Disable;
	adcInit.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
	adcInit.ADC_DataAlign = ADC_DataAlign_Right;
	adcInit.ADC_Resolution = ADC_Resolution_12b;
	adcInit.ADC_NbrOfRegChannel = 1;

	// Prepara o sequencer:
	ADC_CommonInit(ADC2, &adcCommon);
	ADC_Init(ADC2, &adcInit);
	ADC_RegularChannelSequencerLengthConfig(ADC2,1);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_4, 1,ADC_SampleTime_1Cycles5 );
/*
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2,ADC_SampleTime_1Cycles5 );
 */
	// ADC Pronto para rodar.
	ADC_VoltageRegulatorCmd(ADC2, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
}

//
// @fn GetUserAdc()
// @brief toma o resultado da conversao do AD de set point
//
uint16_t GetUserAdc(void)
{
	uint16_t result = 0;
	uint32_t adcFlags = 0;

	//Dispara conversor A/D:
	ADC_StartConversion(ADC2);

	//Aguarda terminar:
	adcFlags = ADC_GetStartConversionStatus(ADC2);
	while(adcFlags != RESET)
	{
		adcFlags = ADC_GetStartConversionStatus(ADC2);
	}

	//Acessa o resultado
	result = ADC_GetConversionValue(ADC2);

	return(result);
}


//
// main()
// @brief funcao principal, captura o estado do botao e regula
//        o brilho do led.
//
int main(void)
{
	uint32_t scan = 0;
	float userVoltage = 5.30f ;


	//Inicializa boost converter:
	BoostInit(5.30f, userVoltage);


	//Configura o botao da discovery para ser usado como controle
	//de dimmer.
	//
	STM_EVAL_PBInit(BUTTON_USER,BUTTON_MODE_GPIO);

	//
	// Configura o systick counter para gerar uma
	// base tempo constante.
	//
	//
	SysTick->CTRL = 0x00;
	SysTick->LOAD = SYSTICK_LOAD_VAL(TICK_FREQ);
	SysTick->CTRL = 0x07;

	scan = tickCounter;

	InitUserAdc();


	//Aciona o boost controller :
	BoostStart();

	for(;;)
	{
		//Escaneia o botao da placa a cada 10ms
		if(tickCounter - scan >= BUTTON_SCAN_TIME)
		{
			//Toma o resultado da conversao:
			userVoltage = ADC_TO_VOLTAGE(GetUserAdc());

			//Atualiza o valor da tensao de saida:
		    BoostVoltageChange(userVoltage);

		    scan = tickCounter;
		}
	}
}

//
// SysTick_Handler()
// @brief esta interrupcao atualiza o contador de
//        base de tempo.
void SysTick_Handler(void)
{
	tickCounter++;
}
