/////////////////////////////////////////////////////////////////////////////
/// debug.c
/// All relevant debugging tasks, including sending data to the PC User Interface
/// via bluetooth
/////////////////////////////////////////////////////////////////////////////
/// Protocol for PC User interface
/// [Startseq][Length][Checksum][ID][Data]
///
/// [Startseq]: ["PCUI_MSG"] (8 chars)
/// [Lenght]: [{b2},{b1_lsb}] (16bit; 2 chars)
/// [checksum]: (Sum of all Data chars) [{b4},{b3},{b2},{b1_lsb}] (32bit; 4 chars)
/// [ID]: (3 chars)
///		["MPD"]: Map Data (13 chars). Rob->PC
///					- resolution (mm) (1byte)
///					- size x (2byte)
///					- size y "
///					- size z (1byte)
///					- rob x  (2byte)
///					- rob y  "
///					- rob z  (1byte)
///					- dir    (2byte)
///		["MAP"]: Map transmission (size y + 3 chars). Rob->PC
///					- current stage (1byte)
///					- current line (transmission linewise) (2byte)
///					- Pixel (size x byte)
///		["WAY"]: Waypoints (n waypoints * 7 chars). Bidirectional
///					- ID (1byte)
///					- x  (2byte)
///					- y  "
///					- z  "
///		["STA"]: Status (1 char). Bidirectional.
///					- start/stop
///	[Data]: [Lenght] chars


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
#include "outf.h"
#include "comm_api.h"
#include "comm.h"
#include "xv11.h"
#include "navigation_api.h"

QueueHandle_t xQueueTXUSART2;
QueueHandle_t xQueueRXUSART2;

// ============================================================================
portTASK_FUNCTION( vDebugTask, pvParameters ) {
	portTickType xLastWakeTime;
	//portBASE_TYPE xStatus;
	//UBaseType_t uxHighWaterMark;

	/* The parameters are not used. */
	( void ) pvParameters;

	xLastWakeTime = xTaskGetTickCount();

	xQueueTXUSART2 = xQueueCreate( 1500, sizeof(char));
	if( xQueueTXUSART2 == 0 )
		foutf(&error, "xQueueTXUSART2 COULD NOT BE CREATED!\n");
	xQueueRXUSART2 = xQueueCreate( 200, sizeof(char));
	if( xQueueRXUSART2 == 0 )
		foutf(&error, "xQueueRXUSART2 COULD NOT BE CREATED!\n");

	foutf(&debugOS, (const char *)"xTask DEBUG started.\n");

	for(;;)
	{
		if(slamUI.active)
		{
			pcui_sendWaypoints(nav_wpStart);
			pcui_sendMap(&slam);
			pcui_processReceived();
		}
		else
		{
			vTaskDelayUntil( &xLastWakeTime, ( 500 / portTICK_RATE_MS ) );
		}
	}
}

void USART2_IRQHandler(void) //PCUI Receive...
{
	static BaseType_t usart2tx_ISRnewDat = pdFALSE;
	static BaseType_t usart2rx_ISRnewDat = pdFALSE;

	// check if the USART2 receive interrupt flag was set
	if( USART_GetITStatus(USART2, USART_IT_RXNE) )
	{
		STM_EVAL_LEDToggle(LED5);
		u_int8_t data = USART2->DR;
		if(xQueueRXUSART2 != 0)
			xQueueSendToBackFromISR(xQueueRXUSART2, &data, &usart2rx_ISRnewDat);
	}

	if((USART_GetFlagStatus(USART2, USART_FLAG_TC) == SET) && (USART_GetITStatus (USART2, USART_IT_TXE) == SET)) //Ready to send next byte and tx interrupt active?
	{
		u_int8_t data;
		if(xQueueReceiveFromISR(xQueueTXUSART2, &data, &usart2tx_ISRnewDat)) //Send byte from Queue if available
			USART_SendData(USART2, data);
		else
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE); //otherwise disable tx interrupt
	}
}

//////////////////////////////////////////////////////////////////////////////
/// \brief pcui_sendMsg
///			Sends a message (definition: see protocol) via bluetooth to the
///			PC (including calculating and sending checksum)
/// \param id
///			ID of the message (see protocol)
/// \param length
///			Length of message (in bytes)
/// \param msg
///			Message
///
void pcui_sendMsg(char *id, u_int32_t length, char *msg)
{
	int32_t checksum = 0;
	char len[2];
	char chk[4];

	len[0] = (char) (length & 0x00ff);
	len[1] = (char) ((length & 0xff00) >> 8);

	for(u_int32_t i = 0; i < length; i++)
		checksum += msg[i];

	chk[0] = (char) (checksum & 0x000000ff);
	chk[1] = (char) ((checksum & 0x0000ff00) >> 8);
	chk[2] = (char) ((checksum & 0x00ff0000) >> 16);
	chk[3] = (char) ((checksum & 0xff000000) >> 24);

	out_puts_l(&slamUI, "PCUI_MSG", 8); //Startseq
	out_puts_l(&slamUI, len, 2);
	out_puts_l(&slamUI, chk, 4);
	out_puts_l(&slamUI, id, 3);
	out_puts_l(&slamUI, msg, length);
}

//////////////////////////////////////////////////////////////////////////////
/// \brief pcui_sendMap
///			Sends the map to the computer. Also sends the map frame data and
///			the robot position!
/// \param slam
///			Pointer to slam container
///

u8 sm_sendMap = 0; //As statemachine, otherwise it would take too much time
u8 sendMap_z = 0;
int16_t sendMap_y = 0;

void pcui_sendMap(slam_t *slam)
{
	char mpd[13]; //MaPData message container array
	char buf[(MAP_SIZE_X_MM / MAP_RESOLUTION_MM) + 3];

	switch(sm_sendMap)
	{
	case 0:

		mpd[0] = MAP_RESOLUTION_MM;
		mpd[1] = (MAP_SIZE_X_MM & 0xff);
		mpd[2] = (MAP_SIZE_X_MM & 0xff00) >> 8;
		mpd[3] = (MAP_SIZE_Y_MM & 0xff);
		mpd[4] = (MAP_SIZE_Y_MM & 0xff00) >> 8;
		mpd[5] = MAP_SIZE_Z_LAYERS;
		mpd[6] = ((int16_t)slam->robot_pos.coord.x & 0xff); //Has to be converted from float to integer
		mpd[7] = ((int16_t)slam->robot_pos.coord.x & 0xff00) >> 8;
		mpd[8] = ((int16_t)slam->robot_pos.coord.y & 0xff);
		mpd[9] = ((int16_t)slam->robot_pos.coord.y & 0xff00) >> 8;
		mpd[10] = slam->robot_pos.coord.z;
		mpd[11] = ((int16_t)slam->robot_pos.psi & 0xff);
		mpd[12] = ((int16_t)slam->robot_pos.psi & 0xff00) >> 8;

		out_puts_l(&slamUI, "\e[0m", 5); //VT100: clear all colorsettings
		pcui_sendMsg((char *)"MPD", 13, mpd); //Send mapdata

		sm_sendMap = 1;
		break;
	case 1:
		/// Send a message for each line of the map. If we send the whole map, we would calculate the
		/// checksum and be ready one second after that - in the meantime, the map would have changed
		/// and the checksum does not matches anymore. Therefore, we save the current line (y), transmit it
		/// with the line information and the matching checksum and receive it as message on the pc. If the
		/// checksum does not matches there, we simply ignore the line and go on.

		buf[0] = sendMap_z; //Current stage to send
		buf[1] = sendMap_y & 0xff; //Current line to send
		buf[2] = (sendMap_y & 0xff00) >> 8;

		for(int i = 0; i < (MAP_SIZE_X_MM / MAP_RESOLUTION_MM); i++) //Map information itself beginning in byte 3
			buf[i + 3] = slam->map.px[i][sendMap_y][sendMap_z];

		pcui_sendMsg((char *)"MAP", (MAP_SIZE_X_MM / MAP_RESOLUTION_MM) + 3, buf); //Send line

		sendMap_y ++;
		if(sendMap_y == (MAP_SIZE_Y_MM / MAP_RESOLUTION_MM))
		{
			sendMap_y = 0;
			sendMap_z ++;
			if(sendMap_z == MAP_SIZE_Z_LAYERS)
			{
				sendMap_z = 0;
				sm_sendMap = 0;
			}
		}
		break;
	default:
		sm_sendMap = 0;
		break;
	}
}

void pcui_sendWaypoints(nav_waypoint_t *start)
{
	if(nav_wpAmount > 0)
	{
		/// One waypoint contains:
		/// x (2 bytes)
		/// y (2 bytes)
		/// z (1 byte)
		/// id (2 bytes)
		/// id_next (2 bytes)
		/// id last (2 bytes)
		/// -> 11 bytes per waypoint

		char wpdata[11 * nav_wpAmount];

		nav_waypoint_t *wp = start;

		for(u8 i = 0; i < nav_wpAmount; i++)
		{
			int16_t wp_next_id = -1;
			int16_t wp_prev_id = -1;
			if(wp->next != NULL)
				wp_next_id = wp->next->id;
			if(wp->previous != NULL)
				wp_next_id = wp->previous->id;

			wpdata[(i * 11) + 0] = wp->x & 0xff;
			wpdata[(i * 11) + 1] = (wp->x & 0xff00) >> 8;
			wpdata[(i * 11) + 2] = wp->y & 0xff;
			wpdata[(i * 11) + 3] = (wp->y & 0xff00) >> 8;
			wpdata[(i * 11) + 4] = wp->z;
			wpdata[(i * 11) + 5] = wp->id & 0xff;
			wpdata[(i * 11) + 6] = (wp->id & 0xff00) >> 8;
			wpdata[(i * 11) + 7] = wp_next_id & 0xff;
			wpdata[(i * 11) + 8] = (wp_next_id & 0xff00) >> 8;
			wpdata[(i * 11) + 9] = wp_prev_id & 0xff;
			wpdata[(i * 11) + 10] = (wp_prev_id & 0xff00) >> 8;
		}

		pcui_sendMsg((char *)"LWP", 11 * nav_wpAmount, wpdata); //Send line
	}
}

void pcui_processReceived(void)
{
	u_int8_t data;
	xQueueReceive(xQueueRXUSART2, &data, 0); //Blocks until new data arrive
}

// Simply print to the debug console a string based on the type of reset.
// ============================================================================
void vDebugPrintResetType( void ) {

	if ( PWR_GetFlagStatus( PWR_FLAG_WU ) )
		foutf(&debugOS, "PWR: Wake Up flag\n" );
	if ( PWR_GetFlagStatus( PWR_FLAG_SB ) )
		foutf(&debugOS, "PWR: StandBy flag.\n" );
	if ( PWR_GetFlagStatus( PWR_FLAG_PVDO ) )
		foutf(&debugOS, "PWR: PVD Output.\n" );
	if ( PWR_GetFlagStatus( PWR_FLAG_BRR ) )
		foutf(&debugOS, "PWR: Backup regulator ready flag.\n" );
	if ( PWR_GetFlagStatus( PWR_FLAG_REGRDY ) )
		foutf(&debugOS, "PWR: Main regulator ready flag.\n" );

	if ( RCC_GetFlagStatus( RCC_FLAG_BORRST ) )
		foutf(&debugOS, "RCC: POR/PDR or BOR reset\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_PINRST ) )
		foutf(&debugOS, "RCC: Pin reset.\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_PORRST ) )
		foutf(&debugOS, "RCC: POR/PDR reset.\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_SFTRST ) )
		foutf(&debugOS, "RCC: Software reset.\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_IWDGRST ) )
		foutf(&debugOS, "RCC: Independent Watchdog reset.\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_WWDGRST ) )
		foutf(&debugOS, "RCC: Window Watchdog reset.\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_LPWRRST ) )
		foutf(&debugOS, "RCC: Low Power reset.\n" );
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
	//USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // enable the USART2 receive interrupt



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
