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
#include "timers.h"

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

static void vTimerSendData(TimerHandle_t xTimer);
u8 sendOnce = 0;

// ============================================================================
portTASK_FUNCTION( vDebugTask, pvParameters ) {
	portTickType xLastWakeTime;

	TimerHandle_t xTimerSendData = NULL;

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

	xTimerSendData = xTimerCreate((const char *)"TM_DEB", 50 / portTICK_PERIOD_MS,
												/* This is a periodic timer, so
												xAutoReload is set to pdTRUE. */
												pdTRUE,
												/* The ID is not used, so can be set
												to anything. */
												( void * ) 0,
												/* The callback function that switches
												the LED off. */
												vTimerSendData
											);

	/* Start the created timer.  A block time of zero is used as the timer
		command queue cannot possibly be full here (this is the first timer to
		be created, and it is not yet running). */
	xTimerStart(xTimerSendData, 0);

	foutf(&debugOS, (const char *)"xTask DEBUG started.\n");

	for(;;)
	{
		if(slamUI.active)
		{
			//pcui_sendMap(&slam);

			pcui_processReceived();
		}
		else
		{
			sendOnce = 0;
			vTaskDelayUntil( &xLastWakeTime, ( 500 / portTICK_RATE_MS ) );
		}
	}
}

static void vTimerSendData(TimerHandle_t xTimer)
{
	if(slamUI.active && !sendOnce)
	{
		pcui_sendWaypoints(nav_wpStart); //Send waypoints
		pcui_sendMapdata(&slam);
		sendOnce = 1;
	}
}

void USART2_IRQHandler(void) //PCUI Receive...
{
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// check if the USART2 receive interrupt flag was set
	if( USART_GetITStatus(USART2, USART_IT_RXNE) )
	{
		u_int8_t data = USART2->DR;
		if(xQueueRXUSART2 != 0)
			xQueueSendToBackFromISR(xQueueRXUSART2, &data, &xHigherPriorityTaskWoken);
	}

	if((USART_GetFlagStatus(USART2, USART_FLAG_TC) != RESET))// && (USART_GetITStatus (USART2, USART_IT_TXE) != RESET)) //Ready to send next byte and tx interrupt active?
	{
		u_int8_t data;
		if(xQueueReceiveFromISR(xQueueTXUSART2, &data, &xHigherPriorityTaskWoken)) //Send byte from Queue if available
			USART_SendData(USART2, data);
		else
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE); //otherwise disable tx interrupt
	}

/*	if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
			{
		  if( xQueueReceiveFromISR( TxQueue, &ch, &xHigherPriorityTaskWoken ) )
			{
			  USART_SendData(USART1, ch);
			}else{
			   //disable Transmit Data Register empty interrupt
			   USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
				 }
			}*/

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
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

u8 sendMap_z = 0;
int16_t sendMap_y = 0;

void pcui_sendMap(slam_t *slam)
{
	char buf[(MAP_SIZE_X_MM / MAP_RESOLUTION_MM) + 3];

	/// Send a message for each line of the map. If we send the whole map, we would calculate the
	/// checksum and be ready one second after that - in the meantime, the map would have changed
	/// and the checksum does not matches anymore. Therefore, we save the current line (y), transmit it
	/// with the line information and the matching checksum and receive it as message on the pc. If the
	/// checksum does not matches there, we simply ignore the line and go on.

	for(u8 z = 0; z < MAP_SIZE_Z_LAYERS; z++)
	{
		for(int16_t y = 0; y < (MAP_SIZE_Y_MM / MAP_RESOLUTION_MM); y ++)
		{
			buf[0] = z; //Current stage to send
			buf[1] = y & 0xff; //Current line to send
			buf[2] = (y & 0xff00) >> 8;

			for(int i = 0; i < (MAP_SIZE_X_MM / MAP_RESOLUTION_MM); i++) //Map information itself beginning in byte 3
				buf[i + 3] = slam->map.px[i][y][z];

			pcui_sendMsg((char *)"MAP", (MAP_SIZE_X_MM / MAP_RESOLUTION_MM) + 3, buf); //Send line
		}
	}
	/*buf[0] = sendMap_z; //Current stage to send
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
		}
	}*/
}

void pcui_sendWaypoints(nav_waypoint_t *start)
{
	/// [Waypoint amount (2 bytes)]<Waypoint amount>*[Waypoint]
	/// One waypoint contains:
	/// x (2 bytes)
	/// y (2 bytes)
	/// z (1 byte)
	/// id (2 bytes)
	/// id last (2 bytes) //We only need the id of the last checkpoint because we transfer them in the order they are connected
	/// -> 9 bytes per waypoint + 2 for whole msg

	char wpdata[(9 * nav_wpAmount) + 2];

	nav_waypoint_t *wp;
	int16_t i;

	wpdata[0] = nav_wpAmount & 0xff; //Store amount of waypoints
	wpdata[1] = (nav_wpAmount & 0xff00) >> 8;

	for(wp = start, i = 0; wp != NULL; wp = wp->next, i++) //Transmit in the order they are connected!!!
	{
		int16_t wp_prev_id = -1;
		if(wp->previous != NULL)
			wp_prev_id = wp->previous->id;

		wpdata[(i * 9) + 2] = wp->x & 0xff;
		wpdata[(i * 9) + 3] = (wp->x & 0xff00) >> 8;
		wpdata[(i * 9) + 4] = wp->y & 0xff;
		wpdata[(i * 9) + 5] = (wp->y & 0xff00) >> 8;
		wpdata[(i * 9) + 6] = wp->z;
		wpdata[(i * 9) + 7] = wp->id & 0xff;
		wpdata[(i * 9) + 8] = (wp->id & 0xff00) >> 8;
		wpdata[(i * 9) + 9] = wp_prev_id & 0xff;
		wpdata[(i * 9) + 10] = (wp_prev_id & 0xff00) >> 8;
	}

	pcui_sendMsg((char *)"LWP", (9 * nav_wpAmount) + 2, wpdata); //Send message
}

void pcui_sendMapdata(slam_t *slam)
{
	char mpd[13]; //MaPData message container array

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

	//out_puts_l(&slamUI, "\e[0m", 5); //VT100: clear all colorsettings
	pcui_sendMsg((char *)"MPD", 13, mpd); //Send mapdata
}


u8 sm_RXgetStart = 0;
u8 rx_getStart(char c)
{
	u8 retVar = 0;

	switch(sm_RXgetStart)
	{
	case 0:
		//sm_RXgetStart = (c == 'P') ? sm_RXgetStart+1 : 0;
		if(c == 'P')
			sm_RXgetStart ++;
		else
			sm_RXgetStart = 0;
		break;
	case 1:
		if(c == 'C')
			sm_RXgetStart ++;
		else
			sm_RXgetStart = 0;
		break;
	case 2:
		if(c == 'U')
			sm_RXgetStart ++;
		else
			sm_RXgetStart = 0;
		break;
	case 3:
		if(c == 'I')
			sm_RXgetStart ++;
		else
			sm_RXgetStart = 0;
		break;
	case 4:
		if(c == '_')
			sm_RXgetStart ++;
		else
			sm_RXgetStart = 0;
		break;
	case 5:
		if(c == 'M')
			sm_RXgetStart ++;
		else
			sm_RXgetStart = 0;
		break;
	case 6:
		if(c == 'S')
			sm_RXgetStart ++;
		else
			sm_RXgetStart = 0;
		break;
	case 7:
		if(c == 'G')
			retVar = 1;

		sm_RXgetStart = 0;
		break;
	default: sm_RXgetStart = 0; break;
	}

	return retVar;
}

u8 compareID(char *msg, const char * msgcomp)
{
	if((msg[0] == msgcomp[0]) &&
	   (msg[1] == msgcomp[1]) &&
	   (msg[2] == msgcomp[2]))
		return 1;
	else
		return 0;
}

u8 sm_prcRX = 0;
int16_t msg_len = 0;
int32_t msg_chk = 0;
int32_t msg_chk_computed = 0;
char msg_id[3];
int16_t msgBufCount = 0;
char msgBuf[512];

void processLWP()
{
	/// One waypoint contains:
	/// x (2 bytes)
	/// y (2 bytes)
	/// z (1 byte)
	/// id (2 bytes)
	/// id prev (2 bytes)
	/// -> 9 bytes per waypoint

	nav_initWaypointStack(); //Reset Waypoint list

	int16_t amount = msgBuf[0] + (msgBuf[1] << 8); //Don’t write value into nav_wpAmount!!!! It’s handled automatically in nav_attachWaypoint
	nav_waypoint_t w;

	for(int i = 0; i < amount; i ++) //The list is transmitted in the order they are linked!
	{
		w.x = (msgBuf[(i * 9) + 2] + (msgBuf[(i * 9) + 3] << 8));
		w.y = (msgBuf[(i * 9) + 4] + (msgBuf[(i * 9) + 5] << 8));
		w.z = (msgBuf[(i * 9) + 6]);
		w.id = (msgBuf[(i * 9) + 7] + (msgBuf[(i * 9) + 8] << 8));
		int wpID_prev = msgBuf[(i * 9) + 9] + (msgBuf[(i * 9) + 10] << 8);
		if(wpID_prev != -1 && i != 0) //There is a waypoint in the list before this one, otherwise it represents the start of the list
		{
			w.previous = nav_getWaypoint(wpID_prev);
			w.previous->next = &w;
		}
		//nav_wpStart automatically initalized in attachWaypoint (if it is the first one)
		nav_attachWaypoint(&w);
	}
}

void pcui_processReceived(void)
{
	u_int8_t data;
	if(xQueueReceive(xQueueRXUSART2, &data, 0))
	{
		switch(sm_prcRX)
		{
		case 0:	if(rx_getStart(data))				sm_prcRX ++;
				break;
		case 1:	msg_len = data;						sm_prcRX ++;    break;  //Lenght (2 bytes)
		case 2:	msg_len += (int16_t)(data << 8);
				if(msg_len < 512) //The length is not checked by the checksum. In case there is transmitted something wrong (and it IS, if the message is this long) abort already here.
					sm_prcRX ++;
				else
					sm_prcRX = 0;

				break;
		case 3:	msg_chk = data;						sm_prcRX ++;    break; //Checksum (4 bytes)
		case 4:	msg_chk += (int16_t)(data << 8);	sm_prcRX ++;    break;
		case 5:	msg_chk += (int32_t)(data << 16);	sm_prcRX ++;    break;
		case 6:	msg_chk += (int32_t)(data << 24);
				sm_prcRX ++;
				break;
		case 7:	msg_id[0] = data;					sm_prcRX ++;    break; //ID (3 bytes/chars)
		case 8:	msg_id[1] = data;					sm_prcRX ++;    break;
		case 9:	msg_id[2] = data;
				msg_chk_computed = 0;
				msgBufCount = 0;
				sm_prcRX ++;
				break;

		case 10: //Buffer Message
				if(msgBufCount < msg_len)
				{
					msgBuf[msgBufCount] = data;
					msg_chk_computed += msgBuf[msgBufCount];
					msgBufCount ++;
				}

				if(msgBufCount == msg_len)
				{
					if(msg_chk_computed == msg_chk) //Checksum matches!
					{
						if(compareID(msg_id, (const char *)"LWP"))
							processLWP();
					}

					sm_prcRX = 0;
				}

				break;
		default: sm_prcRX = 0; break;
		}
	}
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
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // enable the USART2 receive interrupt



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
