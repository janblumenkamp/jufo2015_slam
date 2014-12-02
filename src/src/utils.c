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
#include "printf.h"

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
}
// ============================================================================
void vUSART2_Init( void ) {
	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 *
	 * They make our life easier because we don't have to mess around with
	 * the low level stuff of setting bits in the correct registers
	 */

	GPIO_InitTypeDef GPIO_InitStructure; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART2 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* enable APB1 peripheral clock for USART2
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* enable the peripheral clock for the pins used by
	 * USART2, PA2 for TX and PA3 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART2 peripheral
	 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // Pins 2 (TX) and 3 (RX) are used
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOA, &GPIO_InitStructure);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	/* The RX and TX pins are now connected to their AF
	 * so that the USART2 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); //
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	/* Now the USART_InitStruct is used to define the
	 * properties of USART2
	 */
	USART_InitStruct.USART_BaudRate = 460800;				// The baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART2, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting


	/* Here the USART2 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART2_IRQHandler() function
	 * if the USART2 receive interrupt occurs
	 */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // enable the USART3 receive interrupt

	// Configure the NVIC Preemption Priority Bits
	// wichtig!, sonst stimmt nichts überein mit den neuen ST Libs (ab Version 3.1.0)
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;

	// entspricht 11-15, 11 ist das höchst mögliche, sonst gibt es Probleme mit dem OS
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = (configMAX_SYSCALL_INTERRUPT_PRIORITY >> 4) + 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );


	// finally this enables the complete USART2 peripheral
	USART_Cmd(USART2, ENABLE);
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
