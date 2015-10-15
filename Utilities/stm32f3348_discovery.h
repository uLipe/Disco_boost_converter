/**
  ******************************************************************************
  * @file    stm32f3348_discovery.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    20-June-2014
  * @brief   This file contains definitions for STM32F3348 DISCO's Leds, push-buttons
  *          hardware resources.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F3348_DISCOVERY_H
#define __STM32F3348_DISCOVERY_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f30x.h"


/** @addtogroup Utilities
  * @{
  */

/** @addtogroup STM32F3348_DISCOVERY
  * @{
  */
      
/** @addtogroup STM32F3348_DISCOVERY_LOW_LEVEL
  * @{
  */ 

/** @defgroup STM32F3348_DISCOVERY_LOW_LEVEL_Exported_Types
  * @{
  */
typedef enum 
{
  LED3 = 0,
  LED4 = 1,
  LED5 = 2,
  LED6 = 3
} Led_TypeDef;

typedef enum 
{  
  BUTTON_USER = 0,
} Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;

/**
  * @}
  */ 

/** @defgroup STM32F3348_DISCOVERY_LOW_LEVEL_Exported_Constants
  * @{
  */ 

/** @addtogroup STM32F3348_DISCOVERY_LOW_LEVEL_LED
  * @{
  */
#define LEDn                             4

#define LED3_PIN                         GPIO_Pin_6
#define LED3_GPIO_PORT                   GPIOB
#define LED3_GPIO_CLK                    RCC_AHBPeriph_GPIOB

#define LED4_PIN                         GPIO_Pin_8
#define LED4_GPIO_PORT                   GPIOB
#define LED4_GPIO_CLK                    RCC_AHBPeriph_GPIOB
  
#define LED5_PIN                         GPIO_Pin_9
#define LED5_GPIO_PORT                   GPIOB
#define LED5_GPIO_CLK                    RCC_AHBPeriph_GPIOB  

#define LED6_PIN                         GPIO_Pin_7
#define LED6_GPIO_PORT                   GPIOB
#define LED6_GPIO_CLK                    RCC_AHBPeriph_GPIOB

/**
  * @}
  */                                                                                                        
    
/** @addtogroup STM32F3348_DISCOVERY_LOW_LEVEL_BUTTON
  * @{
  */  
#define BUTTONn                          1

/**
 * @brief User & Wakeup push-button
 */
#define USER_BUTTON_PIN                GPIO_Pin_0
#define USER_BUTTON_GPIO_PORT          GPIOA
#define USER_BUTTON_GPIO_CLK           RCC_AHBPeriph_GPIOA
#define USER_BUTTON_EXTI_LINE          EXTI_Line0
#define USER_BUTTON_EXTI_PORT_SOURCE   EXTI_PortSourceGPIOA
#define USER_BUTTON_EXTI_PIN_SOURCE    EXTI_PinSource0
#define USER_BUTTON_EXTI_IRQn          EXTI0_IRQn 
/**
  * @}
  */

/**
  * @}
  */

/** @defgroup STM32F3348_DISCOVERY_LOW_LEVEL_Exported_Functions
  * @{
  */
void STM_EVAL_LEDInit(Led_TypeDef Led);
void STM_EVAL_LEDOn(Led_TypeDef Led);
void STM_EVAL_LEDOff(Led_TypeDef Led);
void STM_EVAL_LEDToggle(Led_TypeDef Led);
void STM_EVAL_PBInit(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode);
uint32_t STM_EVAL_PBGetState(Button_TypeDef Button);

/**
  * @}
  */
  
#ifdef __cplusplus
}
#endif

#endif /* __STM32F3348_DISCOVERY_H */
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
