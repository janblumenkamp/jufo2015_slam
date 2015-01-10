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
#include "outf.h"

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
