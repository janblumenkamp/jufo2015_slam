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

// ============================================================================
portTASK_FUNCTION( vDebugTask, pvParameters ) {
	portTickType xLastWakeTime;
	//portBASE_TYPE xStatus;
	//UBaseType_t uxHighWaterMark;

	/* The parameters are not used. */
	( void ) pvParameters;

	xLastWakeTime = xTaskGetTickCount();

	foutf(&debugOS, (const char *)"xTask DEBUG started.\n");

	for(;;)
	{
		foutf(&debugOS, "Watermark debug: %i\n", uxTaskGetStackHighWaterMark( NULL ));

		if(slamUI.active)
			pcui_sendMap(&slam);

		vTaskDelayUntil( &xLastWakeTime, ( 20 / portTICK_RATE_MS ) );
	}
}

void USART2_IRQHandler(void) //PCUI Receive...
{
	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART2, USART_IT_RXNE) )
	{
		STM_EVAL_LEDToggle(LED5);
		char data = USART2->DR;
		data *= 2;
		//Process...
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

		//out_puts_l(&slamUI, "\e[0m", 5); //VT100: clear all colorsettings
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

		//out_puts_l(&slamUI, "\e[0m", 5); //VT100: clear all colorsettings
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


// Simply print to the debug console a string based on the type of reset.
// ============================================================================
void vDebugPrintResetType( void ) {

	if ( PWR_GetFlagStatus( PWR_FLAG_WU ) )
		foutf(&debugOS, "PWR: Wake Up flag\r\n" );
	if ( PWR_GetFlagStatus( PWR_FLAG_SB ) )
		foutf(&debugOS, "PWR: StandBy flag.\r\n" );
	if ( PWR_GetFlagStatus( PWR_FLAG_PVDO ) )
		foutf(&debugOS, "PWR: PVD Output.\r\n" );
	if ( PWR_GetFlagStatus( PWR_FLAG_BRR ) )
		foutf(&debugOS, "PWR: Backup regulator ready flag.\r\n" );
	if ( PWR_GetFlagStatus( PWR_FLAG_REGRDY ) )
		foutf(&debugOS, "PWR: Main regulator ready flag.\r\n" );

	if ( RCC_GetFlagStatus( RCC_FLAG_BORRST ) )
		foutf(&debugOS, "RCC: POR/PDR or BOR reset\r\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_PINRST ) )
		foutf(&debugOS, "RCC: Pin reset.\r\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_PORRST ) )
		foutf(&debugOS, "RCC: POR/PDR reset.\r\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_SFTRST ) )
		foutf(&debugOS, "RCC: Software reset.\r\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_IWDGRST ) )
		foutf(&debugOS, "RCC: Independent Watchdog reset.\r\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_WWDGRST ) )
		foutf(&debugOS, "RCC: Window Watchdog reset.\r\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_LPWRRST ) )
		foutf(&debugOS, "RCC: Low Power reset.\r\n" );
}
