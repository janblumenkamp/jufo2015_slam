//--------------------------------------------------------------
// File     : stm32_ub_pwm_tim4.c
// Datum    : 26.03.2013
// Version  : 1.0
// Autor    : UB
// EMail    : mc-4u(@)t-online.de
// Web      : www.mikrocontroller-4u.de
// CPU      : STM32F4
// IDE      : CooCox CoIDE 1.7.0
// Module   : GPIO, TIM
// Funktion : PWM-Funktionen per Timer4
//
// Hinweis  : mögliche Pinbelegungen
//            CH1 : [PB6, PD12]
//            CH2 : [PB7, PD13]
//            CH3 : [PB8, PD14]
//            CH4 : [PB9, PD15]
//--------------------------------------------------------------


//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "stm32_ub_pwm_tim4.h"



//--------------------------------------------------------------
// interne Funktionen
//--------------------------------------------------------------
void P_PWM_InitIO(void);
void P_PWM_InitTIM(void);


//--------------------------------------------------------------
// Definition aller PWM Pins
// Reihenfolge wie bei PWM_TIM4_NAME_t
//
// Channel : [1...4]
// Init = Startwert : [0...PWM_TIM4_PERIODE]
//--------------------------------------------------------------
PWM_TIM4_t PWM_TIM4[] = {
  // Name     ,Channel, PORT , PIN       , CLOCK               , Source         , Init
  {PWM_T4_PD12,1      ,GPIOD ,GPIO_Pin_12,RCC_AHB1Periph_GPIOD ,GPIO_PinSource12,  64},
  {PWM_T4_PD14,3      ,GPIOD ,GPIO_Pin_14,RCC_AHB1Periph_GPIOD ,GPIO_PinSource14,  128},
};



//--------------------------------------------------------------
// init und start vom PWM mit Timer4
//-------------------------------------------------------------- 
void UB_PWM_TIM4_Init(void)
{
  P_PWM_InitIO();
  P_PWM_InitTIM();
}


//--------------------------------------------------------------
// stellt einen PWM-Wert für einen Kanal ein 
// wert : [0 bis PWM_TIM4_PERIODE]
//--------------------------------------------------------------
void UB_PWM_TIM4_SetPWM(PWM_TIM4_NAME_t pwm, uint16_t wert)
{
  if(wert>PWM_TIM4_PERIODE) wert=PWM_TIM4_PERIODE;

  if(PWM_TIM4[pwm].CHANNEL==1) {
    TIM4->CCR1 = wert;
  } 
  if(PWM_TIM4[pwm].CHANNEL==2) {
    TIM4->CCR2 = wert;
  } 
  if(PWM_TIM4[pwm].CHANNEL==3) {
    TIM4->CCR3 = wert;
  } 
  if(PWM_TIM4[pwm].CHANNEL==4) {
    TIM4->CCR4 = wert;
  } 
}


//--------------------------------------------------------------
// interne Funktion
// Init aller IO-Pins
//--------------------------------------------------------------
void P_PWM_InitIO(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  PWM_TIM4_NAME_t pwm_name;

  for(pwm_name=0;pwm_name<PWM_TIM4_ANZ;pwm_name++) {
    // Clock Enable
    RCC_AHB1PeriphClockCmd(PWM_TIM4[pwm_name].PWM_CLK, ENABLE);

    // Config des Pins als Digital-Ausgang
    GPIO_InitStructure.GPIO_Pin = PWM_TIM4[pwm_name].PWM_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(PWM_TIM4[pwm_name].PWM_PORT, &GPIO_InitStructure);

    // Alternative-Funktion mit dem IO-Pin verbinden
    GPIO_PinAFConfig(PWM_TIM4[pwm_name].PWM_PORT, PWM_TIM4[pwm_name].PWM_SOURCE, GPIO_AF_TIM4);
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
  PWM_TIM4_NAME_t pwm_name;
  uint16_t wert;

  // Clock enable
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 

  // Timer init
  TIM_TimeBaseStructure.TIM_Period = PWM_TIM4_PERIODE;
  TIM_TimeBaseStructure.TIM_Prescaler = PWM_TIM4_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);


  for(pwm_name=0;pwm_name<PWM_TIM4_ANZ;pwm_name++) {
    if(PWM_TIM4[pwm_name].CHANNEL==1) {
      // Channel 1
      TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
      TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
      wert=PWM_TIM4[pwm_name].PWM_INIT;
      if(wert>PWM_TIM4_PERIODE) wert=PWM_TIM4_PERIODE;
      TIM_OCInitStructure.TIM_Pulse = wert;
      TIM_OCInitStructure.TIM_OCPolarity = PWM_TIM4_POLARITY;
      TIM_OC1Init(TIM4, &TIM_OCInitStructure);
      TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    }
    if(PWM_TIM4[pwm_name].CHANNEL==2) {
      // Channel 2
      TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
      TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
      wert=PWM_TIM4[pwm_name].PWM_INIT;
      if(wert>PWM_TIM4_PERIODE) wert=PWM_TIM4_PERIODE;
      TIM_OCInitStructure.TIM_Pulse = wert;
      TIM_OCInitStructure.TIM_OCPolarity = PWM_TIM4_POLARITY;
      TIM_OC2Init(TIM4, &TIM_OCInitStructure);
      TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    }
    if(PWM_TIM4[pwm_name].CHANNEL==3) {
      // Channel 3
      TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
      TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
      wert=PWM_TIM4[pwm_name].PWM_INIT;
      if(wert>PWM_TIM4_PERIODE) wert=PWM_TIM4_PERIODE;
      TIM_OCInitStructure.TIM_Pulse = wert;
      TIM_OCInitStructure.TIM_OCPolarity = PWM_TIM4_POLARITY;
      TIM_OC3Init(TIM4, &TIM_OCInitStructure);
      TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    }
    if(PWM_TIM4[pwm_name].CHANNEL==4) {
      // Channel 4
      TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
      TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
      wert=PWM_TIM4[pwm_name].PWM_INIT;
      if(wert>PWM_TIM4_PERIODE) wert=PWM_TIM4_PERIODE;
      TIM_OCInitStructure.TIM_Pulse = wert;
      TIM_OCInitStructure.TIM_OCPolarity = PWM_TIM4_POLARITY;
      TIM_OC4Init(TIM4, &TIM_OCInitStructure);
      TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
    }
  }

  // Timer enable
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
}
