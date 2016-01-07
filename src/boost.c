//
// 			SERIE DIGITAL POWER COM A DISCOVERY EMBARCADOS
//
//	@file boost.c
//  @brief Arquivo contendo modulo de operacao de boost converter, pode rodar em
//         modo openloop ou closed loop usando a estrategia voltage mode controller
//
//
#include "stm32f30x.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_comp.h"
#include "stm32f30x_hrtim.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_dac.h"
#include "stm32f30x_adc.h"
#include "boost.h"
//
// Constantes de configuracao, internas e hardware:
//

//
// Sobre o HRTIM:
//
#define BOOST_PWM_L_PIN      GPIO_Pin_10 //diodo sincrono
#define BOOST_PWM_H_PIN      GPIO_Pin_11 //low side
#define BOOST_IN_PIN         GPIO_Pin_8 //low side


#define BOOST_PWM_PORT       GPIOA      //
#define BOOST_HRTIM_LOAD_VAL(x) 	((x == 0?0:((SystemCoreClock/x) * 64))) //VAlor de recarga do HRTIM

//
// sobre o VFB apenas em modo closed loop:
//
#define BOOST_VFB_PIN       GPIO_Pin_3
#define BOOST_VIN_PIN       GPIO_Pin_1
#define BOOST_VFB_PORT      GPIOA

//
// estados de operacao do conversor boost:
//
#define BOOST_RUN  0x00000001
#define BOOST_HALT 0x00000002
#define BOOST_OFF  0x00000008


//
// Limites de operacao da arimetica usada no controle:
//
#define PID_MAX_VAL  45.0f
#define PID_MIN_VAL -45.0f

//
// Mapeia a saida do controle PID de -32.0f <---> +32.0f para
// Controlador PWM entre 0 - 18K
//
#define MAX_PWM    16384.0f
#define MIN_PWM    16.3f


#define MAP_SCALE  ((MAX_PWM - MIN_PWM) / (PID_MAX_VAL - 0.0f))
#define MAP_TO_PWM(x) (uint16_t)((x + 0.1f) * MAP_SCALE)



//
// Rampa de subida do softstart:
//
#define BOOST_RAMP_TIME  20 //Hz
#define BOOST_RAMP_TICKS BOOST_SW_FREQ
#define BOOST_TICKS_PER_TIME (BOOST_RAMP_TICKS / BOOST_RAMP_TIME)
#define BOOST_SOFT_START_PWM_PERCENT ((50.0f/100.0f) * MAX_PWM)


#define BOOST_SOFT_START_PWM_INC (uint16_t)(((BOOST_SOFT_START_PWM_PERCENT) \
											 / BOOST_TICKS_PER_TIME) + 1.0f )


#define BOOST_SOFT_START_PWM_VAL (uint16_t)BOOST_SOFT_START_PWM_PERCENT

//
// Variaveis estaticas:
//
static float transferRatio = 0.0f;
static const float vfbScale   = 3.1187e-3f; // Scale = (Vref / ADC_Steps) * Vbus to Vfb ratio
static float inVoltage  = 0.0f;
static float outVoltage  = 0.0f;
static float vfbVal      = 0.0f;


static uint16_t dutyCicle  = 0;
static uint32_t boostFlags = BOOST_OFF;
static uint32_t adcStep = 0;
static bool  inSoftStart = false;




struct loopcontrolwork
{
	float *pfeed;
	float *perr;
	float *kp;
	float *ki;
	float *kd;
	float *integrator;
	float *prevError;
	float *pOut;
};



//variaveis da malha de controle:
struct loopcontrolwork loopwork;

float kp = 0.0f;
float ki = 0.0f;
float kd = 0.0f;
float integrator = 0.0f;
float prevError = 0.0f;
float output = 0;


//
// forward references as funcoes usadas internamente:
//

#if BOOST_CTL_TYPE == 1

extern void DigitalPowerLoopFunc(void);
void DigitalPowerLoopSetCoef(float kp, float ki, float kd);

#endif

void Boost_HW_SetAnalog(void);
void Boost_HW_SetPwm(void);
void Boost_HW_SetInterrupts(void);
void Boost_HW_SetDutyCicle(uint16_t dc);
void HRTIM1_TIMB_IRQHandler(void);
void ADC1_2_IRQHandler(void);

//
// codigos de erro do buck converter:
//
typedef enum
{
	knoError = 0,
	koutVoltageTooLow = -1,
	koutVoltageTooHigh,
	kboostIsDiscontinuous,

}errorcode;



//
// Funcoes internas:
//

//
// Boost_HW_SetAnalog
//
void Boost_HW_SetAnalog(void)
{
	GPIO_InitTypeDef analogIo, vin;
	analogIo.GPIO_Mode = GPIO_Mode_AN;
	analogIo.GPIO_Pin = BOOST_VFB_PIN;

	vin.GPIO_Mode = GPIO_Mode_AN;
	vin.GPIO_Pin = BOOST_VIN_PIN;


	//Aciona clock do port onde esta localizado o A/D
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	//Inicializa o pino de IO desejado:
	GPIO_Init(BOOST_VFB_PORT, &analogIo);
	GPIO_Init(BOOST_VFB_PORT, &vin);

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
	ADC_DeInit(ADC1);
	ADC_CommonInit(ADC1, &adcCommon);
	ADC_Init(ADC1, &adcInit);
	ADC_RegularChannelSequencerLengthConfig(ADC1,1);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1,ADC_SampleTime_1Cycles5 );

	// ADC Pronto para rodar.
	//ADC_VoltageRegulatorCmd(ADC1, ENABLE);

	uint32_t i = 0;

	//Aguarda o vreg estabilizar.
	for( i = 0 ; i < 0x7FFF; i++);

	ADC_Cmd(ADC1, ENABLE);
}

//
// Boost_HW_SetPwm()
//
void Boost_HW_SetPwm(void)
{
	GPIO_InitTypeDef gpioL, gpioH, gpioIn;
	gpioL.GPIO_Pin  = BOOST_PWM_L_PIN;
	gpioL.GPIO_Mode  = GPIO_Mode_AF;
	gpioL.GPIO_OType = GPIO_OType_PP;
	gpioL.GPIO_Speed = GPIO_Speed_Level_3;

	gpioH.GPIO_Pin  = BOOST_PWM_H_PIN;
	gpioH.GPIO_Mode  = GPIO_Mode_AF;
	gpioH.GPIO_OType = GPIO_OType_PP;
	gpioH.GPIO_Speed = GPIO_Speed_Level_3;

	gpioIn.GPIO_Pin  = BOOST_IN_PIN;
	gpioIn.GPIO_Mode  = GPIO_Mode_OUT;
	gpioIn.GPIO_OType = GPIO_OType_PP;
	gpioIn.GPIO_Speed = GPIO_Speed_Level_3;


	//Aciona o clock do HRTIM:
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_HRTIM1, ENABLE);
	RCC_HRTIM1CLKConfig(RCC_HRTIM1CLK_PLLCLK);

	//Aciona o pino que queremos o HRTIM:
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_Init(BOOST_PWM_PORT, &gpioL);
	GPIO_PinAFConfig(BOOST_PWM_PORT, GPIO_PinSource10,GPIO_AF_13);
	GPIO_Init(BOOST_PWM_PORT, &gpioH);
	GPIO_PinAFConfig(BOOST_PWM_PORT, GPIO_PinSource11,GPIO_AF_13);

	GPIO_Init(BOOST_PWM_PORT, &gpioIn);
	GPIO_ResetBits(BOOST_PWM_PORT, BOOST_IN_PIN);

	//COnfigura o HRTIM para obter-se uma saida pushpull em pwmH e L:
	// OBS usamos o timer B
	HRTIM1->HRTIM_MASTER.MCR  = 0x00000000;								  //desliga o HRTIM
	HRTIM1->HRTIM_MASTER.MPER = BOOST_HRTIM_LOAD_VAL(BOOST_SW_FREQ);	  //Periodo do master timer
	HRTIM1->HRTIM_TIMERx[1].TIMxCR = 0x00000000 | ( 1 << 3);			  //MOdo de operacao como output compare e em push-pull
	HRTIM1->HRTIM_TIMERx[1].PERxR  = BOOST_HRTIM_LOAD_VAL(BOOST_SW_FREQ); //TPWM em hertz acertado
	HRTIM1->HRTIM_TIMERx[1].CMP1xR = MAP_TO_PWM(32.0f);				  //Duty cicle minimo
	HRTIM1->HRTIM_TIMERx[1].SETx1R = (1 << 2);							  //acerta evento de set
	HRTIM1->HRTIM_TIMERx[1].RSTx1R = (1 << 3);							  //e evento de reset
	//HRTIM1->HRTIM_TIMERx[1].SETx2R = (1 << 2);
	//HRTIM1->HRTIM_TIMERx[1].RSTx2R = (1 << 3);
	HRTIM1->HRTIM_TIMERx[1].DTxR  =  (250 << 16) | (250 << 0) | (0x01 << 10);			  //acerta o dead time para 20ns (obtido experim.)
	HRTIM1->HRTIM_TIMERx[1].OUTxR  = (1 << 8);							  //acerta o sistema de push pull
	HRTIM1->HRTIM_COMMON.OENR      = (1 << 2) | (1 << 3);				  //habilita as saidas PWM.

	HRTIM1->HRTIM_MASTER.MCR  = 0x003F0008; 							  //Habilita o master timer
}

//
// Boost_HW_SetInterrupts()
//
void Boost_HW_SetInterrupts(void)
{
	//Desliga o HRTIM1 e ADC1 como fonte de interrupcao:
	NVIC_DisableIRQ(HRTIM1_TIMB_IRQn);
    NVIC_DisableIRQ(ADC1_2_IRQn);

	//Aciona interupcao por reset (ela que sera o trigger para disparar
	//a conversao A/D)
	HRTIM_ITConfig(HRTIM1, 0x01, HRTIM_TIM_IT_RST, ENABLE);

	//Aciona a interrupcao do ADC:
	ADC_ITConfig(ADC1, ADC_IT_EOC,ENABLE);
}

//
// Boost_HW_SetDutyCicle()
//
void Boost_HW_SetDutyCicle(uint16_t dc)
{
	HRTIM1->HRTIM_TIMERx[1].CMP1xR = dc;
}

//
// BoostError()
//
void BoostError(errorcode err)
{
	//
	// Eh uma funcao interna mas util para debug
	// se cair aqui o programa pede um halt da fonte
	// e deixa o errorcode visivel para debug.
	if(err != knoError)
	{
		BoostHalt();
		while(err);
	}
}


//
// DigitalPowerLoopSetCoef()
//
#if BOOST_CTL_TYPE == 1
void DigitalPowerLoopSetCoef(float _kp, float _ki, float _kd)
{

	integrator = 0.0f;
	prevError  = 0.0f;

	//prepara o set de coeficientes:
	kp = _kp;
	ki = _ki;
	kd = _kd;


	//liga a array de estados nos nodes da funcao de controle:
	loopwork.pOut    = &output;
	loopwork.perr    = &outVoltage;
	loopwork.pfeed   = &vfbVal;

	loopwork.kp = &kp;
	loopwork.ki = &ki;
	loopwork.kd = &kd;
	loopwork.integrator = &integrator;
	loopwork.prevError = &prevError;

}
#endif

//
// DigitalPowerSaturate()
//

#if BOOST_CTL_TYPE == 1
void DigitalPowerSaturate(void)
{
	//satura resultado:
	if(output > PID_MAX_VAL) output = PID_MAX_VAL;
	if(output < PID_MIN_VAL) output = PID_MIN_VAL;

	//satura integrador:
	if(integrator > PID_MAX_VAL) integrator = PID_MAX_VAL;
	if(integrator < PID_MIN_VAL) integrator = PID_MIN_VAL;

	//satura erro anterior:
	if(prevError > PID_MAX_VAL) prevError = PID_MAX_VAL;
	if(prevError < PID_MIN_VAL) prevError = PID_MIN_VAL;

}
#endif


//
// HTRIM ISR:
//
void HRTIM1_TIMB_IRQHandler(void)
{

#if BOOST_CTL_TYPE == 1

	if(true != inSoftStart)
	{

		//Roda a malha de controle:
		DigitalPowerLoopFunc();

		//Satura:
		DigitalPowerSaturate();

		if(output < 0.0f) output = 0.0f;

		//Prepara o dutycicle para correcao no proximo ciclo:
		dutyCicle = MAP_TO_PWM(output);
	}
	else
	{
		//
		// Se o flag de subida suave estiver ativo, ignora a malha de
		// controle:
		//
		dutyCicle += BOOST_SOFT_START_PWM_INC;
		if(dutyCicle > BOOST_SOFT_START_PWM_VAL)
		{
			inSoftStart = false;
		}

	}
#endif
	//Dispara a leitura do conversor A/D:
	ADC_StartConversion(ADC1);

	//Atualiza novo DutyCicle calculado previamente:
	Boost_HW_SetDutyCicle(dutyCicle);


	//Limpa o flag de interrupcao:
	HRTIM_ClearITPendingBit(HRTIM1, 0x01, 0xFFFFFFFF);
}


//
// ADC1 ISR:
//
void ADC1_2_IRQHandler(void)
{
	//Limpa fonte de interupcao:
	ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);


	//Toma o resultado da conversao A/D
	vfbVal = vfbScale * (float)ADC_GetConversionValue(ADC1);

}

//
// conversor boost, funcoes publicas:
//

//
// BoostInit()
//
void BoostInit(float inputVoltage, float outputVoltage)
{


	//
	// Checa se a relacao Vo/Vi eh valida:
	//
	if(inputVoltage > outputVoltage)
	{
		errorcode  err = koutVoltageTooLow;
		BoostError(err);
	}

	//
	// Checa se a tensao de saida desejada eh valida:
	//
	if(outputVoltage > BOOST_MAX_VOLTAGE)
	{
		errorcode err = koutVoltageTooHigh;
		BoostError(err);
	}

#if BOOST_CTL_TYPE == 0

	//
	// Determina o duty cicle
	// nota, estamos trabalhando com um buck em mondo continuo
	// Vo/Vi = 1 / (1 - D)
	transferRatio = 1.0f - (inputVoltage / outputVoltage);
	dutyCicle = (uint16_t)(transferRatio * (float)BOOST_HRTIM_LOAD_VAL(BOOST_SW_FREQ));
	inVoltage = inputVoltage;
	outVoltage= outputVoltage;

#else
	//Malha fechada, seleciona o setpoint inicial
	//E ignora a relacao VO/VI, sera compensado automaticamente.
	//E configura o PID:
	inVoltage  = inputVoltage;
	outVoltage = outputVoltage;
	if(inVoltage > BOOST_MAX_VOLTAGE) inVoltage = BOOST_MAX_VOLTAGE;

	DigitalPowerLoopSetCoef(15.0, 0.80, 0.1);
#endif

	//
	// Inicializa o hw do boost converter mas sem operar:
	//
	Boost_HW_SetAnalog();
	Boost_HW_SetPwm();
	Boost_HW_SetInterrupts();

	//
	// Sistema inicialmente em halt:
	//
	BoostHalt();
	boostFlags = BOOST_HALT;
}

//
// BoostStart()
//
void BoostStart(void)
{

#if BOOST_CTL_TYPE == 0
	Boost_HW_SetDutyCicle(dutyCicle);
#else
	const float tmp = 0.0f;
	dutyCicle = MAP_TO_PWM(tmp);
	Boost_HW_SetDutyCicle(dutyCicle);
#endif
	//
	// Com o dutycicle pre calculado da um trigger no pwm
	//
	HRTIM1->HRTIM_MASTER.MCR = 0x003F0008;
	Boost_HW_SetInterrupts();

	//Libera VIN:
	GPIO_SetBits(BOOST_PWM_PORT, BOOST_IN_PIN);
	boostFlags = BOOST_RUN;


	NVIC_SetPriority(HRTIM1_TIMB_IRQn, 250);
	NVIC_SetPriority(ADC1_2_IRQn, 251);


    NVIC_EnableIRQ(HRTIM1_TIMB_IRQn);
    NVIC_EnableIRQ(ADC1_2_IRQn);

}

//
// BoostVoltageChange()
//
void BoostVoltageChange(float newVoltage)
{

	// Checa se a relacao Vo/Vi eh valida:
	//
	if(inVoltage > newVoltage)
	{
		errorcode  err = koutVoltageTooLow;
		BoostError(err);
	}

	//
	// Checa se a tensao de saida desejada eh valida:
	//
	if(newVoltage > BOOST_MAX_VOLTAGE)
	{
		errorcode err = koutVoltageTooHigh;
		BoostError(err);
	}


#if BOOST_CTL_TYPE == 0	 //apenas calcula o dc em caso de malha aberta

	//
	// Recalcula parametros do boost:
	//
	transferRatio = 1.0f - (inVoltage / newVoltage);
	dutyCicle = (uint16_t)(transferRatio * (float)BOOST_HRTIM_LOAD_VAL(BOOST_SW_FREQ));



	//Malha fechada, o PID se encarrega disso:
	outVoltage = newVoltage;

	//
	// Modifica tensao de saida:
	//
	Boost_HW_SetDutyCicle(dutyCicle);
#else
	//COntrole em malha fechada, apenas modifica o setpoint
	//E ignora a relacao VO/VI, sera compensado por SW.
	outVoltage = newVoltage;
#endif

}

//
// BoostHalt()
//
void BoostHalt(void)
{
	if(boostFlags & BOOST_HALT)
	{
		//
		// Ja esta inoperante.
		//
		return;
	}

	//
	// derruba o master hrtim:
	//
	HRTIM1->HRTIM_MASTER.MCR = 0;

	//Derruba VIN
	GPIO_ResetBits(BOOST_PWM_PORT, BOOST_IN_PIN);

	//
	// derruba as interupcoes:
	//
	NVIC_DisableIRQ(HRTIM1_TIMB_IRQn);
	NVIC_DisableIRQ(ADC1_2_IRQn);

	boostFlags = BOOST_HALT;
	inSoftStart = false;
	dutyCicle = 0;

}

