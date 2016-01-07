//
// 			SERIE DIGITAL POWER COM A DISCOVERY EMBARCADOS
//
//	@file boost.h
//  @brief Arquivo contendo modulo de operacao de boost converter, pode rodar em
//         modo openloop ou closed loop usando a estrategia voltage mode controller
//
//
#ifndef __BOOST_H
#define __BOOST_H

#include <stdbool.h>
#include "stdint.h"

//
// Configuracao do conversor:
//
#define BOOST_SW_FREQ     			250000 	//Frequencia do PWM em HZ
#define BOOST_MAX_VOLTAGE	   		15.0f 	//Maxima tensao do boost em [V]
#define BOOST_VFB_MAX_VOLTAGE       2.98f    //Maxima tensao que pode ser lida pelo feed
#define BOOST_CTL_TYPE              1      // 0 - openLoop
											// 1 - VMC closed loop

//
// API para inicializacao e start do conversor:
//

//
// @fn BoostInit()
// @brief Inicializa o conversor boost sincrono da discovery
//
void BoostInit(float inputVoltage, float outputVoltage);

//
// @fn BoostStart()
// @brief depois de configurado o usuario pode chamar essa funcao
//        pra disparar a operacao do boost
void BoostStart(void);

//
// @fn BoostVoltageChange()
// @brief Modifica a tensao de saida do boost converter:
//
void BoostVoltageChange(float newVoltage);

//
// @fn BoostHalt()
// @brief como o boost usado eh sincrono o user pode chamar essa funcao
//        para derrubar a operacao do boost e impor 0.0v na saida independente
//        da entrada.
void BoostHalt(void);







#endif
