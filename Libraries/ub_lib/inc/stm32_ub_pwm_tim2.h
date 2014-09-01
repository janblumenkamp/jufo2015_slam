//--------------------------------------------------------------
// File     : stm32_ub_pwm_tim2.h
//--------------------------------------------------------------

//--------------------------------------------------------------
#ifndef __STM32F4_UB_PWM_TIM2_H
#define __STM32F4_UB_PWM_TIM2_H


//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"


//--------------------------------------------------------------
// Liste aller PWM Pins
// (keine Nummer doppelt und von 0 beginnend)
//--------------------------------------------------------------
typedef enum 
{
  PWM_T2_PB11 = 0,  // PWM per TIM2 an PB11
  PWM_T2_PA2  = 1   // PWM per TIM2 an PA2
}PWM_TIM2_NAME_t;

#define  PWM_TIM2_ANZ   2 // Anzahl von PWM_TIM2_NAME_t


//--------------------------------------------------------------
// PWM Einstellungen (Frequenz und Auflösung)
// periode   = Auflösung (max = 0xFFFF => 16bit)
// prescaler = Frquenz
//
// Grundfrequenz = 2*APB1 (APB1=42MHz) => TIM_CLK=84MHz
// periode   : 0 bis 0xFFFF
// prescale  : 0 bis 0xFFFF
//
// PWM-Frq = TIM_CLK/(periode+1)/(vorteiler+1)
//--------------------------------------------------------------
#define  PWM_TIM2_PERIODE   0xFF // periode   (0xFF => 8bit)
#define  PWM_TIM2_PRESCALE   327 // prescaler (328 => 1kHz)


//--------------------------------------------------------------
// PWM Setting (Polarität)
//
// Hi => Hi-Impulse
// Lo => Lo-Impulse
//--------------------------------------------------------------
#define  PWM_TIM2_POLARITY  TIM_OCPolarity_High
//#define  PWM_TIM2_POLARITY  TIM_OCPolarity_Low


//--------------------------------------------------------------
// Struktur eines PWM Kanals
//--------------------------------------------------------------
typedef struct {
  PWM_TIM2_NAME_t PWM_NAME; // Name
  const uint8_t CHANNEL;    // Channel [1...4]
  GPIO_TypeDef* PWM_PORT;   // Port
  const uint16_t PWM_PIN;   // Pin
  const uint32_t PWM_CLK;   // Clock
  const uint8_t PWM_SOURCE; // Source
  const uint16_t PWM_INIT;  // Init
}PWM_TIM2_t;


//--------------------------------------------------------------
// Globale Funktionen
//--------------------------------------------------------------
void UB_PWM_TIM2_Init(void);
void UB_PWM_TIM2_SetPWM(PWM_TIM2_NAME_t pwm, uint16_t wert);



//--------------------------------------------------------------
#endif // __STM32F4_UB_PWM_TIM2_H
