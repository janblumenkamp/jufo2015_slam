/*
 * utils.c
 *
 *  Created on: Jan 1, 2012
 *      Author: James
 */

#include "FreeRTOS.h"
#include "task.h"

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "utils.h"

#include <stdarg.h>
#include <ctype.h>


// ----------------------------------------------------------------------------
void HwInit( void ) {
    SystemCoreClockUpdate( );
    // Make sure SysTick is running at a 1ms rate.
    if ( SysTick_Config( SystemCoreClock / 1000 ) ) {
        /* Capture error */
        while ( 1 );
    }
    // SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK_Div8 );

    /* Initialize Leds mounted on STM32F4-Discovery board */
    STM_EVAL_LEDInit(LED4);
    STM_EVAL_LEDInit(LED3);
    STM_EVAL_LEDInit(LED5);
    STM_EVAL_LEDInit(LED6);

    /* Turn on LED4 and LED5 */
    STM_EVAL_LEDOn(LED4);
    STM_EVAL_LEDOn(LED5);

    vUSART2_Init();   // Start up UART2
}
// ============================================================================
void vUSART2_Init( void ) {
	USART_ClockInitTypeDef USART_ClockInitStruct;
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable needed clocks for uart.
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2, ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE );

	// Make sure you use 'GPIO_PinSource2' and NOT 'GPIO_Pin_2'.  Using the
	// latter will not work!
	GPIO_PinAFConfig( GPIOA, GPIO_PinSource2, GPIO_AF_USART2 );
	GPIO_PinAFConfig( GPIOA, GPIO_PinSource3, GPIO_AF_USART2 );

	// Setup Tx / Rx pins.
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;			// Tx Pin
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init( GPIOA, &GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;			// Rx Pin
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init( GPIOA, &GPIO_InitStructure );

	// Make sure syncro clock is turned off.
	USART_ClockStructInit( &USART_ClockInitStruct );
	USART_ClockInit( USART2, &USART_ClockInitStruct  );

	// Setup transmit complete irq.
	USART_ITConfig( USART2, USART_IT_TC, ENABLE );

	// Use defaults (except baud rate).
	USART_StructInit( &USART_InitStructure );
	USART_InitStructure.USART_BaudRate = 460800;
	USART_Init( USART2, &USART_InitStructure );
	USART_Cmd( USART2, ENABLE );

    // Enable USART2 interrupt.
    /*
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    */
}

////////////////////////////////////////////////////////////////////////////////
/// Sorts the elements of the given array by their size from the smallest to the
/// biggest and returns a pointer to the element at @get (helpfull when used as
/// median-filter)
///
/// Param:
/// @cnt:		Number of elements in @data[] (size in words)
/// @*data: 	Pointer to the first element of the array to sort
/// @get:		Number of the element to return in the sorted array
///
/// @return: Adress of the element at @get int the given, sorted array
////////////////////////////////////////////////////////////////////////////////

int16_t *get_sorted(u8 cnt, int16_t *data, u8 get)
{
	for(u8 i = 0; i <= cnt-1; i ++)
		for(u8 j = i+1; j < cnt; j ++)
			if(data[i] > data[j])
			{
				u16 data_temp = data[i];
				data[i] = data[j];
				data[j] = data_temp;
			}

	return &data[get];
}
