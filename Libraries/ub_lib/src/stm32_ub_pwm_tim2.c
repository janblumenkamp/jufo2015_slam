//--------------------------------------------------------------
// File     : stm32_ub_pwm_tim2.c
// Datum    : 26.03.2013
// Version  : 1.0
// Autor    : UB
// EMail    : mc-4u(@)t-online.de
// Web      : www.mikrocontroller-4u.de
// CPU      : STM32F4
// IDE      : CooCox CoIDE 1.7.0
// Module   : GPIO, TIM
// Funktion : PWM-Funktionen per Timer2
//
// Hinweis  : mögliche Pinbelegungen
//            CH1 : [PA0, PA5]
//            CH2 : [PA1, PB3]
//            CH3 : [PA2, PB10]
//            CH4 : [PA3, PB11]
//--------------------------------------------------------------


//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "stm32_ub_pwm_tim2.h"



//--------------------------------------------------------------
// interne Funktionen
//--------------------------------------------------------------
void P_PWM_InitIO(void);
void P_PWM_InitTIM(void);


//--------------------------------------------------------------
// Definition aller PWM Pins
// Reihenfolge wie bei PWM_TIM2_NAME_t
//
// Channel : [1...4]
// Init = Startwert : [0...PWM_TIM2_PERIODE]
//--------------------------------------------------------------
PWM_TIM2_t PWM_TIM2[] = {
  // Name     ,Channel, PORT , PIN       , CLOCK               , Source         , Init
  {PWM_T2_PB11,4      ,GPIOB ,GPIO_Pin_11,RCC_AHB1Periph_GPIOB ,GPIO_PinSource11,  64},
  {PWM_T2_PA2 ,3      ,GPIOA ,GPIO_Pin_2 ,RCC_AHB1Periph_GPIOA ,GPIO_PinSource2 ,  128},
};



//--------------------------------------------------------------
// init und start vom PWM mit Timer2
//-------------------------------------------------------------- 
void UB_PWM_TIM2_Init(void)
{
  P_PWM_InitIO();
  P_PWM_InitTIM();
}


//--------------------------------------------------------------
// stellt einen PWM-Wert für einen Kanal ein 
// wert : [0 bis PWM_TIM2_PERIODE]
//--------------------------------------------------------------
void UB_PWM_TIM2_SetPWM(PWM_TIM2_NAME_t pwm, uint16_t wert)
{
  if(wert>PWM_TIM2_PERIODE) wert=PWM_TIM2_PERIODE;

  if(PWM_TIM2[pwm].CHANNEL==1) {
    TIM2->CCR1 = wert;
  } 
  if(PWM_TIM2[pwm].CHANNEL==2) {
    TIM2->CCR2 = wert;
  } 
  if(PWM_TIM2[pwm].CHANNEL==3) {
    TIM2->CCR3 = wert;
  } 
  if(PWM_TIM2[pwm].CHANNEL==4) {
    TIM2->CCR4 = wert;
  } 
}


//--------------------------------------------------------------
// interne Funktion
// Init aller IO-Pins
//--------------------------------------------------------------
void P_PWM_InitIO(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  PWM_TIM2_NAME_t pwm_name;

  for(pwm_name=0;pwm_name<PWM_TIM2_ANZ;pwm_name++) {
    // Clock Enable
    RCC_AHB1PeriphClockCmd(PWM_TIM2[pwm_name].PWM_CLK, ENABLE);

    // Config des Pins als Digital-Ausgang
    GPIO_InitStructure.GPIO_Pin = PWM_TIM2[pwm_name].PWM_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(PWM_TIM2[pwm_name].PWM_PORT, &GPIO_InitStructure);

    // Alternative-Funktion mit dem IO-Pin verbinden
    GPIO_PinAFConfig(PWM_TIM2[pwm_name].PWM_PORT, PWM_TIM2[pwm_name].PWM_SOURCE, GPIO_AF_TIM2);
  }  
}


//--------------------------------------------------------------
// interne Funktion
// Init vom Timer
//--------------------------------------------------------------
void P_PWM_InitTIM(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  PWM_TIM2_NAME_t pwm_name;
  uint16_t wert;

  // Clock enable
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 

  // Timer init
  TIM_TimeBaseStructure.TIM_Period = PWM_TIM2_PERIODE;
  TIM_TimeBaseStructure.TIM_Prescaler = PWM_TIM2_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);


  for(pwm_name=0;pwm_name<PWM_TIM2_ANZ;pwm_name++) {
    if(PWM_TIM2[pwm_name].CHANNEL==1) {
      // Channel 1
      TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
      TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
      wert=PWM_TIM2[pwm_name].PWM_INIT;
      if(wert>PWM_TIM2_PERIODE) wert=PWM_TIM2_PERIODE;
      TIM_OCInitStructure.TIM_Pulse = wert;
      TIM_OCInitStructure.TIM_OCPolarity = PWM_TIM2_POLARITY;
      TIM_OC1Init(TIM2, &TIM_OCInitStructure);
      TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    }
    if(PWM_TIM2[pwm_name].CHANNEL==2) {
      // Channel 2
      TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
      TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
      wert=PWM_TIM2[pwm_name].PWM_INIT;
      if(wert>PWM_TIM2_PERIODE) wert=PWM_TIM2_PERIODE;
      TIM_OCInitStructure.TIM_Pulse = wert;
      TIM_OCInitStructure.TIM_OCPolarity = PWM_TIM2_POLARITY;
      TIM_OC2Init(TIM2, &TIM_OCInitStructure);
      TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    }
    if(PWM_TIM2[pwm_name].CHANNEL==3) {
      // Channel 3
      TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
      TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
      wert=PWM_TIM2[pwm_name].PWM_INIT;
      if(wert>PWM_TIM2_PERIODE) wert=PWM_TIM2_PERIODE;
      TIM_OCInitStructure.TIM_Pulse = wert;
      TIM_OCInitStructure.TIM_OCPolarity = PWM_TIM2_POLARITY;
      TIM_OC3Init(TIM2, &TIM_OCInitStructure);
      TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    }
    if(PWM_TIM2[pwm_name].CHANNEL==4) {
      // Channel 4
      TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
      TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
      wert=PWM_TIM2[pwm_name].PWM_INIT;
      if(wert>PWM_TIM2_PERIODE) wert=PWM_TIM2_PERIODE;
      TIM_OCInitStructure.TIM_Pulse = wert;
      TIM_OCInitStructure.TIM_OCPolarity = PWM_TIM2_POLARITY;
      TIM_OC4Init(TIM2, &TIM_OCInitStructure);
      TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    }
  }

  // Timer enable
  TIM_ARRPreloadConfig(TIM2, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
}




