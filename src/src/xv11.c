#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "stm32f4xx_conf.h"
#include "xv11.h"
#include "main.h"
#include "utils.h"
#include "outf.h"
#include "stm32_ub_pwm_tim3.h"
#include "slam.h"
#include "slamdefs.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "stm32f4_discovery.h"

volatile XV11_t xv11;

volatile slam_coordinates_t scanStart_lastRobPos;

//Private Function Prototypes

/* This funcion initializes the USART1 peripheral
 *
 * Arguments: baudrate --> the baudrate at which the USART is
 * 						   supposed to operate
 */

int8_t xv11_state(u8 state)
{
	if(state == XV11_STARTING)
	{
		xv11.state = XV11_STARTING;
		UB_PWM_TIM3_SetPWM(PWM_T3_PB5, XV11_SPEED_PWM_INIT); //make sure motor is spinning
	}
	else if(state == XV11_OFF)
		xv11.state = XV11_OFF;

	return xv11.state;
}

void xv11_init(void)
{
	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 *
	 * They make our life easier because we don't have to mess around with
	 * the low level stuff of setting bits in the correct registers
	 */
	GPIO_InitTypeDef GPIO_InitStructure; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* enable APB2 peripheral clock for USART1
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* enable the peripheral clock for the pins used by
	 * USART1, PB6 for TX and PB7 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStructure);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); //
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

	/* Now the USART_InitStruct is used to define the
	 * properties of USART1
	 */
	USART_InitStruct.USART_BaudRate = 115200;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting


	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

	// Configure the NVIC Preemption Priority Bits
	// wichtig!, sonst stimmt nichts überein mit den neuen ST Libs (ab Version 3.1.0)
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;

	// entspricht 11-15, 11 ist das höchst mögliche, sonst gibt es Probleme mit dem OS
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;//(configMAX_SYSCALL_INTERRUPT_PRIORITY >> 4) + 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);

	//Init Motor PWM:
	UB_PWM_TIM3_Init();
	UB_PWM_TIM3_SetPWM(PWM_T3_PB5, 0);

	xv11_state(XV11_OFF);
}

enum SM_XV11 {
	OFF, INIT_SEARCHSTART, SEARCHSTART,
	GETPACKAGE, PROCESS
};

enum XV11_PACKAGE_DATA {
	STARTBYTE, INDEX,
	SPEED_LSB, SPEED_MSB,
	D0_B0, D0_B1, D0_B2, D0_B3,
	D1_B0, D1_B1, D1_B2, D1_B3,
	D2_B0, D2_B1, D2_B2, D2_B3,
	D3_B0, D3_B1, D3_B2, D3_B3,
	CHK_LSB, CHK_MSB
};

// this is the interrupt request handler (IRQ) for ALL USART1 interrupts
void USART1_IRQHandler(void)
{
	static BaseType_t slamTaskWoken = pdFALSE; //Synchronisation between SLAM Task and Lidar ISR

	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART1, USART_IT_RXNE) )
	{
		static u_int8_t sm = INIT_SEARCHSTART;
		static u_int16_t xv11_dist_index = 0;
		static float xv11_speedreg_i = XV11_SPEED_IREG_INIT; //I-Regulator (speed)
		static u_int8_t xv11_state_on_cnt = 0; //Before the state switches to "XV11_ON", the rpm has to be stable a few iterations
		u_int32_t checksum = 0;
		u_int8_t data = (u_int8_t)USART1->DR; // the character from the USART1 data register is saved in data

		static u_int8_t xv11_package[XV11_PACKAGE_LENGTH];

		//XV-11 Lidar Protocol: (https://github.com/Xevel/NXV11/wiki)
		//<start byte (FA) [1]> <index[1]> <speed[2]> <Data 0[4]> <Data 1[4]> <Data 2[4]> <Data 3[4]> <checksum[2]>

		if(xv11.state == XV11_OFF)
			sm = OFF;

		switch (sm)
		{
		case OFF:
			UB_PWM_TIM3_SetPWM(PWM_T3_PB5, 0); //power down motor
			if(xv11.state == XV11_STARTING)
				sm = INIT_SEARCHSTART;
			break;
		case INIT_SEARCHSTART: //Find start byte of protocoll

			UB_PWM_TIM3_SetPWM(PWM_T3_PB5, XV11_SPEED_PWM_INIT); //make sure motor is spinning

			sm = SEARCHSTART;

		case SEARCHSTART:

			if(data == 0xFA)
				sm = GETPACKAGE;
			else
				break;

		case GETPACKAGE ... (GETPACKAGE + (XV11_PACKAGE_LENGTH - 1)):

			xv11_package[sm - GETPACKAGE] = data;
			sm ++;
			if(sm != (PROCESS + (XV11_PACKAGE_LENGTH - 1)))
				break;

		case (PROCESS + (XV11_PACKAGE_LENGTH - 1)): //Process

			if(xv11_package[STARTBYTE] != 0xFA) //lost package start
			{
				sm = INIT_SEARCHSTART;
			}
			else //compute checksum and compare
			{
				for(u8 i = STARTBYTE; i < CHK_LSB; i += 2)
				{
					checksum = (checksum << 1) + (xv11_package[i] + (xv11_package[i + 1] << 8));
				}
				checksum = (checksum & 0x7FFF) + (checksum >> 15);
				checksum &= 0x7FFF;

				if(checksum == (xv11_package[CHK_LSB] + (xv11_package[CHK_MSB] << 8))) //checksum matches
				{
					xv11_dist_index = (xv11_package[INDEX] - 0xA0) * 4;
					xv11.speed = (xv11_package[SPEED_LSB] | (xv11_package[SPEED_MSB] << 8)) / 64.0;

					//if(xv11_dist_index == 0) //Synchronization var with the slam algorithm
					//	xSemaphoreGiveFromISR( lidarSync, &slamTaskWoken );

					for(u8 i = 0; i < 4; i++)
					{
						xv11.dist_polar[xv11_dist_index + i] = xv11_package[D0_B0 + (i * 4)] + ((xv11_package[D0_B1 + (i * 4)] & 0x3F) << 8);
						if((xv11_package[D0_B1 + (i * 4)] & 0x80) || (xv11_package[D0_B1 + (i * 4)] & 0x40)) //invalid data flag
							xv11.dist_polar[xv11_dist_index + i] = XV11_VAR_NODATA;
					}

					xv11_speedreg_i += ((XV11_SPEED_RPM_TO - xv11.speed) / XV11_SPEED_DIV_I);
					if(xv11_speedreg_i > 255.0) //8bit PWM
						xv11_speedreg_i = 255.0;
					else if(xv11_speedreg_i < XV11_SPEED_MIN)
						xv11_speedreg_i = XV11_SPEED_MIN;

					UB_PWM_TIM3_SetPWM(PWM_T3_PB5, (int) xv11_speedreg_i);

					if((xv11.speed < (XV11_SPEED_RPM_TO - XV11_SPEED_LIM)) ||
						(xv11.speed > (XV11_SPEED_RPM_TO + XV11_SPEED_LIM)))
					{
						xv11_state_on_cnt = 0;
						xv11.state = XV11_STARTING;
					}
					else
					{
						xv11_state_on_cnt ++;

						if(xv11_state_on_cnt > XV11_STATE_ON_CNT)
							xv11.state = XV11_ON;
					}
				}
				else //checksum does not match
				{
					for(u8 i = 4; i < 8; i++)
					{
						xv11.dist_polar[xv11_dist_index + i] = XV11_VAR_NODATA; //clear the distance information of the package with the false checksum
					}
					xv11_dist_index += 4; //In case the next package is also false and this var can’t set new, go to the next package index
					if(xv11_dist_index > 359)
						xv11_dist_index = 0;
					sm = INIT_SEARCHSTART;
				}

				sm = GETPACKAGE;
			}
			break;

		default:
			break;
		}
	}
	//portEND_SWITCHING_ISR(slamTaskWoken);
}

// Task for processing the lidar data
// ----------------------------------------------------------------------------

portTASK_FUNCTION( vLIDARTask, pvParameters ) {
    portTickType xLastWakeTime;
    uint8_t i=0;

	foutf(&debugOS, "xTask LIDAR started.\r\n");

    xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{

		vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_RATE_MS ) );
    }
}
