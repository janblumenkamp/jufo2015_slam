/*
 * debug.c
 *
 *  Created on: Jan. 5, 2012
 *      Author: James Kemp
 */
#include <stdarg.h>
#include <ctype.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

#include "debug.h"
#include "utils.h"
#include "gui.h"
#include "main.h"
#include "printf.h"

// ============================================================================
portTASK_FUNCTION( vDebugTask, pvParameters ) {
	//portBASE_TYPE xStatus;
	//UBaseType_t uxHighWaterMark;

	/* The parameters are not used. */
	( void ) pvParameters;

	#if(configDEBUG_MESSAGES == 1)
		printf("xTask ’DEBUG’ started.\r\n");
	#endif

	for(;;) {

		/*if ( USART_GetFlagStatus( USART2, USART_FLAG_RXNE ) ) {
			ch = USART_ReceiveData( USART2 );
			// Handle Debug Console Commands Here.
			printf( "%i ", (int)ch);

		 switch ( ch ) {

			// Alphabetical list of commands the console debugger responds to.

			case 'm':
				printf( "Mems dump Stopped.\r\n");
				vSetMemsDump( false );
				break;
			case 'M':
				printf( "Mems dump Started.\r\n");
				vSetMemsDump( true );
				break;

			case 'a':
				printf( "AtoD dump Stopped.\r\n");
				//vSetAtoDDump( FALSE );
				break;
			case 'A':
				printf( "AtoD dump Started.\r\n");
				//vSetAtoDDump( TRUE );
				break;

			case 'l':
				printf( "Loop Count Stopped.\r\n");
				//vSetCntLoops( FALSE );
				break;
			case 'L':
				printf( "Loop Count Started.\r\n");
				//vSetCntLoops( TRUE );
				break;

			// Print out how much stack space remains on each task stack.
			case 's':*/
				/*printf( "Remaining space on Task Stack:\r\n" );
				uxHighWaterMark = uxTaskGetStackHighWaterMark( hDebugTask );
				printf( "Debug\t%d\r\n", uxHighWaterMark);
				uxHighWaterMark = uxTaskGetStackHighWaterMark( hTimeTask );
				printf( "Time\t%d\r\n", uxHighWaterMark);
				uxHighWaterMark = uxTaskGetStackHighWaterMark( hMemsTask );
				printf( "LCD\t%d\r\n", uxHighWaterMark);*/
		/*		break;

			// Add general test code here...
			case 't':
				break;

			default:
				break;
			}
		}*/

		taskYIELD();
	}
}

// Simply print to the debug console a string based on the type of reset.
// ============================================================================
void vDebugPrintResetType( void ) {

	if ( PWR_GetFlagStatus( PWR_FLAG_WU ) )
		printf( "PWR: Wake Up flag\r\n" );
	if ( PWR_GetFlagStatus( PWR_FLAG_SB ) )
		printf( "PWR: StandBy flag.\r\n" );
	if ( PWR_GetFlagStatus( PWR_FLAG_PVDO ) )
		printf( "PWR: PVD Output.\r\n" );
	if ( PWR_GetFlagStatus( PWR_FLAG_BRR ) )
		printf( "PWR: Backup regulator ready flag.\r\n" );
	if ( PWR_GetFlagStatus( PWR_FLAG_REGRDY ) )
		printf( "PWR: Main regulator ready flag.\r\n" );

	if ( RCC_GetFlagStatus( RCC_FLAG_BORRST ) )
		printf( "RCC: POR/PDR or BOR reset\r\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_PINRST ) )
		printf( "RCC: Pin reset.\r\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_PORRST ) )
		printf( "RCC: POR/PDR reset.\r\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_SFTRST ) )
		printf( "RCC: Software reset.\r\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_IWDGRST ) )
		printf( "RCC: Independent Watchdog reset.\r\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_WWDGRST ) )
		printf( "RCC: Window Watchdog reset.\r\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_LPWRRST ) )
		printf( "RCC: Low Power reset.\r\n" );
}
