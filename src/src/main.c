/**
 *****************************************************************************
 **
 **  File        : main.c
 **
 **  Abstract    : main function.
 **
 **  Functions   : main
 **
 **  Environment : Atollic TrueSTUDIO(R)
 **                STMicroelectronics STM32F4xx Standard Peripherals Library
 **
 **  Distribution: The file is distributed as is, without any warranty
 **                of any kind.
 **
 **  (c)Copyright Atollic AB.
 **  You may use this file as-is or modify it according to the needs of your
 **  project. Distribution of this file (unmodified or modified) is not
 **  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
 **  rights to distribute the assembled, compiled & linked contents of this
 **  file as part of an application binary file, provided that it is built
 **  using the Atollic TrueSTUDIO(R) toolchain.
 **
 **
 *****************************************************************************
 */

#include <stdint.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include "utils.h"
#include "debug.h"
#include "printf.h"
#include "xv11.h"
#include "main.h"
#include "gui.h"
#include "slam.h"
#include "comm.h"
#include "comm_api.h"

#include "stm32_ub_touch_ADS7843.h"

#include "stm32_ub_pwm_tim3.h"

// Task priorities: Higher numbers are higher priority.
#define mainTIME_TASK_PRIORITY      ( tskIDLE_PRIORITY + 4 )
#define mainDRIVE_TASK_PRIORITY       ( tskIDLE_PRIORITY + 3 )
#define mainSLAM_TASK_PRIORITY       ( tskIDLE_PRIORITY + 2 )
#define mainGUI_TASK_PRIORITY       ( tskIDLE_PRIORITY + 2 )
#define mainDEBUG_TASK_PRIORITY     ( tskIDLE_PRIORITY + 1 )
#define mainINTEGER_TASK_PRIORITY   ( tskIDLE_PRIORITY )

xTaskHandle hTimeTask;
xTaskHandle hDRIVETask;
xTaskHandle hSLAMTask;
xTaskHandle hGUITask;
xTaskHandle hDebugTask;

portTASK_FUNCTION_PROTO( vTimeTask, pvParameters );
portTASK_FUNCTION_PROTO( vDRIVETask, pvParameters );
portTASK_FUNCTION_PROTO( vSLAMTask, pvParameters );
portTASK_FUNCTION_PROTO( vGUITask, pvParameters );
portTASK_FUNCTION_PROTO( vDebugTask, pvParameters );

u_int32_t systemTick=0;        // Counts OS ticks (default = 1000Hz).
u_int32_t u64IdleTicks=0;    // Value of u64IdleTicksCnt is copied once per sec.
u_int32_t u64IdleTicksCnt=0; // Counts when the OS has no task to execute.
u_int16_t u16PWM1=0;

// ============================================================================
int main( void )
{
	//HwInit();
	LCD_ResetDevice();
	UB_Touch_Init();
	comm_init();
	gui_init();
	vUSART2_Init();
	xv11_init();

	printf("\r\n\n\n\n\n\n\n\n");
	printf("–––––––––––––––––––––––\r\n");
	printf("| FreeRTOS v8.0.0 RC2 |\r\n");
	printf("–––––––––––––––––––––––\r\n");
	printf("Jugend Forscht 2015 v1.0\r\n");
	vDebugPrintResetType();


	/* Initialize Leds mounted on STM32F4-Discovery board */
	STM_EVAL_LEDInit(LED3); STM_EVAL_LEDOff(LED3);
	STM_EVAL_LEDInit(LED4); STM_EVAL_LEDOff(LED4);
	STM_EVAL_LEDInit(LED5); STM_EVAL_LEDOff(LED5);
	STM_EVAL_LEDInit(LED6); STM_EVAL_LEDOff(LED4);

	// Tasks get started here...
	xTaskCreate( vTimeTask, "TIME", configMINIMAL_STACK_SIZE,
			NULL, mainTIME_TASK_PRIORITY, &hTimeTask );
	xTaskCreate( vDRIVETask, "DRIVE", configMINIMAL_STACK_SIZE,
			NULL, mainDRIVE_TASK_PRIORITY, &hDRIVETask );
	xTaskCreate( vSLAMTask, "SLAM", configMINIMAL_STACK_SIZE*3,
			NULL, mainSLAM_TASK_PRIORITY, &hSLAMTask );
	xTaskCreate( vGUITask, "GUI", configMINIMAL_STACK_SIZE * 4,
			NULL, mainGUI_TASK_PRIORITY, &hGUITask );
	xTaskCreate( vDebugTask, "DEBUG", configMINIMAL_STACK_SIZE*2,
			NULL, mainGUI_TASK_PRIORITY, &hDebugTask );

	vTaskStartScheduler(); // This should never return.

    // Will only get here if there was insufficient memory to create
    // the idle task.
    for( ;; );  
}

// This task should run every 50ms.  The task will average 50ms over time by
// monitoring the actual time between calls and self adjusting accordingly.
// ---------------------------------------------------------------------------- 
portTASK_FUNCTION( vTimeTask, pvParameters ) {
    portTickType xLastWakeTime;
    uint8_t i=0;

	#if(configDEBUG_MESSAGES == 1)
		printf("xTask TIME started.\r\n");
	#endif

    xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
		STM_EVAL_LEDToggle(LED3);

		ub_touch_handler_50ms();

		// Once per second, copy the number of idle ticks and then
        // reset the rolling counter.
        if ( ++i == 20 ) {
            i = 0;
            u64IdleTicks = u64IdleTicksCnt;
			u64IdleTicksCnt = 0;
        }

		vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_RATE_MS ) );
    }
}

// This FreeRTOS callback function gets called once per tick (default = 1000Hz).
// ---------------------------------------------------------------------------- 
void vApplicationTickHook( void ) {
	++systemTick;
}

// This FreeRTOS call-back function gets when no other task is ready to execute.
// On a completely unloaded system this is getting called at over 2.5MHz!
// ---------------------------------------------------------------------------- 
void vApplicationIdleHook( void ) {
    ++u64IdleTicksCnt;
}

// A required FreeRTOS function.
// ---------------------------------------------------------------------------- 
void vApplicationMallocFailedHook( void ) {
	printf( "Malloc failed!!!\r\n");
	configASSERT( 0 );  // Latch on any failure / error.
}

void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName) {
	(void) pcTaskName;
	(void) pxTask;
	/* Run time stack overflow checking is performed if
		configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
		function is called if a stack overflow is detected. */

	printf( "xTask %s: STACK OVERFLOW DETECTED!!!\r\n", pcTaskName);
	taskDISABLE_INTERRUPTS();
	for(;;);
}
