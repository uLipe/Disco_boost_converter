//
//		INTRODUCAO A DIGITAL POWER COM A STM32F334
//
//		@file  main.c
//      @brief implementa uma engine de controle de brilho de LED closed loop
//             sem usar a CPU.
//



#include "stm32f30x.h"
#include "stm32f3348_discovery.h"
#include "stm32f30x_rcc.h"
#include "boost.h"

//
// Macros e constantes uteis:
//

#define TICK_FREQ      1000   //frequencia de updade do tick.
#define SYSTICK_LOAD_VAL(x) (x == 0?0:(SystemCoreClock) /x)    //recarga do systick
#define BUTTON_SCAN_TIME 10   //periodo de scanning do botao em ms

#define DIMMING_MAX_VAL  15.0f
#define DIMMING_MIN_VAL  5.50f

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
float bright = DIMMING_MIN_VAL;
uint32_t tickCounter = 0;
dim_state dimmingMchn = kdimmingUp;

//
// main()
// @brief funcao principal, captura o estado do botao e regula
//        o brilho do led.
//
int main(void)
{
	uint32_t scan = 0;

	//Inicializa boost converter:
	BoostInit(5.0f, 10.0f );


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


	//Aciona o boost controller :
	BoostStart();

	for(;;)
	{
		//Escaneia o botao da placa a cada 10ms
		if(tickCounter - scan >= BUTTON_SCAN_TIME)
		{
			//Checa se o botao foi pressionado:
			if(STM_EVAL_PBGetState(BUTTON_USER) != 0)
			{
				//Avalia a maquininha de estados:
				switch(dimmingMchn)
				{
					case kdimmingUp:

						bright += 0.5f;
						if(bright > DIMMING_MAX_VAL)
						{
							//Realiza wrap e troca de estado do dimmer
							dimmingMchn = kdimmingDown;
							bright = DIMMING_MAX_VAL;
						}
					break;


					case kdimmingDown:
						bright-= 0.5f;
						if(bright <= DIMMING_MIN_VAL)
						{
							//Realiza wrap e troca de estado do dimmer
							dimmingMchn = kdimmingUp;
							bright = DIMMING_MIN_VAL;
						}
					break;
				}

			//Atualiza valor de tensao no boost
			BoostVoltageChange(bright);

			}

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
