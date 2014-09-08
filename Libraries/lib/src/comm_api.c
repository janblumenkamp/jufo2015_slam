//////////////////////////////////////////////////////////////////////////////////////
/// comm.c - Interface to the Subcontroller
///
/// Protocoll:
///		<Startbyte 0xAB><Register/Command><Batchlength n>(n*<Data>)<Checksumme1><Checksumme2> -> 5 + n Byte pro Paket
///		Slave antwortet genau so. Bei fehlerhaftem Paket antwortet der Slave mit batch = 0 und Register = 255.
///		Batch: Bit 0..6: Batchlänge, Bit 7: Write access
///
///	Register/Command:
/// 0			Status (succeed/error)
/// 1			DIST_BACK_RIGHT LSB
/// 2			DIST_BACK_RIGHT MSB
/// 3			DIST_RIGHT_BACK LSB
/// 4			DIST_RIGHT_BACK MSB
/// 5			DIST_LEFT_BACK  LSB
/// 6			DIST_LEFT_BACK  MSB
/// 7			DIST_BACK_LEFT  LSB
/// 8			DIST_BACK_LEFT  MSB
/// 9			DIST_FRONT_FRONT LSB
/// 10			DIST_FRONT_FRONT MSB
/// 11			DIST_BACK_BACK   LSB
/// 12			DIST_BACK_BACK   MSB
/// 13			ADC 6 LSB
/// 14			ADC 6 MSB
/// 15			DIST_DOWN LSB
/// 16			DIST_DOWN MSB
/// 17			ADC 8 LSB
/// 18			ADC 8 MSB
/// 19			ADC_BATTERY LSB
/// 20			ADC_BATTERY MSB
/// 21			DIST_FRONT_LEFT LSB
/// 22			DIST_FRONT_LEFT MSB
/// 23			DIST_LEFT_FRONT LSB
/// 24			DIST_LEFT_FRONT MSB
/// 25			SENS_IMPASSE_1 LSB
/// 26			SENS_IMPASSE_1 MSB
/// 27			SENS_IMPASSE_1 LSB
/// 28			SENS_IMPASSE_1 MSB
/// 29			DIST_RIGHT_FRONT LSB
/// 30			DIST_RIGHT_FRONT MSB
/// 31			DIST_FRONT_RIGHT LSB
/// 32			DIST_FRONT_RIGHT MSB
/// 33			Batterie in mV LSB
/// 34			Batterie in mV MSB
/// 35			Batterie in %
/// 36			Motor Encoder L LSB
/// 37			Motor Encoder L
/// 38			Motor Encoder L
/// 39			Motor Encoder L MSB
/// 40			Motor Encoder R LSB
/// 41			Motor Encoder R
/// 42			Motor Encoder R
/// 43			Motor Encoder R MSB
/// 44			Motor Geschwindigkeit L ist
/// 45			Motor Geschwindigkeit R ist
/// 46			Motor Geschwindigkeit L soll
/// 47			Motor Geschwindigkeit R soll
/// 48			Motortreiber aktiv
/// 49			LED Modus
/// 50			LED Hue
/// 51			LED Saturation
/// 52			LED Value
///
////////////////////////////////////////////////////////////////////////////////

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include "main.h"
#include "comm_api.h"
#include "printf.h"

comm_msg_t receivedMessage;
uint8_t messageBuffer[COMM_BUFSIZE];
comm_msg_t sendMessage;

uint8_t waitForResponse = 0; //Has to be set to 1 if a package was sent and the master now waits for a response.

////////////////////////////////////////////////////////////////////////
/// \brief comm_init
///		inits comm (usart and other stuff, variables etc...)

void comm_init(void)
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
	USART_InitTypeDef USART_InitStruct; // this is for the USART3 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* enable APB2 peripheral clock for USART3
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* enable the peripheral clock for the pins used by
	 * USART3, PB6 for TX and PB7 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART3 peripheral
	 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // Pins 10 (TX) and 11 (RX) are used
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStructure);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	/* The RX and TX pins are now connected to their AF
	 * so that the USART3 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3); //
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	/* Now the USART_InitStruct is used to define the
	 * properties of USART3
	 */
	USART_InitStruct.USART_BaudRate = 115200;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART3, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting


	/* Here the USART3 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART3_IRQHandler() function
	 * if the USART3 receive interrupt occurs
	 */
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // enable the USART3 receive interrupt

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;		 // we want to configure the USART3 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART3 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART3 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	// finally this enables the complete USART3 peripheral
	USART_Cmd(USART3, ENABLE);

	receivedMessage.data = messageBuffer;
}

//////////////////////////////////////////////////////////////////
/// \brief USART3_IRQHandler
///		listens to the uart and puts a message into a message struct
///		if receives one

static volatile uint8_t comm_sm = 0; //statemachine

void USART3_IRQHandler(void)
{
	if( USART_GetITStatus(USART3, USART_IT_RXNE) )
	{
		static int8_t comm_batch_i = 0;

		/* read UART status register and UART data register */
		unsigned char data = USART3->DR;

		switch (comm_sm) {
		case WAITFORPACKAGE:
			if(data == 0xAB)
				comm_sm = GET_REGISTER;
			break;
		case GET_REGISTER:
			receivedMessage.reg = data;
			comm_sm = GET_BATCH;
			break;
		case GET_BATCH:
			receivedMessage.batch = (data & COMM_BATCH);
			receivedMessage.batch_write = (data & COMM_BATCH_WRITE) >> 7;
			comm_batch_i = 0;

			if(receivedMessage.batch_write)
				comm_sm = GET_DATA; //Write access! Write into registers.
			else
				comm_sm = GET_CHK_LSB; //The master wants ro read. Continue with checksum.

			break;
		case GET_DATA: //Master wants to write in registers
			receivedMessage.data[comm_batch_i] = data;
			comm_batch_i ++;

			if(comm_batch_i <= receivedMessage.batch)
				break;

		case GET_CHK_LSB: //Checksum LSB
			receivedMessage.checksum = data << 8;
			comm_sm = GET_CHK_MSB;
			break;
		case GET_CHK_MSB:
			receivedMessage.checksum |= data;
			//Received the package. Let the slave process and respond to it!
			comm_sm = NEWMESSAGE;
			break;
		case NEWMESSAGE: break; //BUSY, wait for processing of query. Only listen for new packages if processed and answered.
		default: comm_sm = WAITFORPACKAGE;
			break;
		}
	}
}

//////////////////////////////////////////////////////////////////////////////
/// \brief comm_calcChecksum
///		calculates checksum of the given message and returns it (does not save
///		it into the msg.checksum element!!!)
/// \param msg
/// \return checksum

uint16_t comm_calcChecksum(comm_msg_t *msg)
{
	uint16_t checksum = 0xAB + msg->reg + ((msg->batch_write << 7) | msg->batch);
	if(msg->batch_write)
	{
		for(uint8_t i = 0; i < msg->batch; i++)
		{
			checksum += msg->data[i];
		}
	}
	return checksum;
}

/////////////////////////////////////////////////////////////////////////////////
/// \brief comm_sendPackage
///		sends the message/the package *msg to the master and also calculates checksum etc.
/// \param msg
///		message to send

void comm_sendPackage(comm_msg_t *msg)
{
	comm_sendByte(0xAB);
	comm_sendByte(msg->reg);
	comm_sendByte((msg->batch_write << 7) | msg->batch);
	if(msg->batch_write)
	{
		for(uint8_t i = 0; i < msg->batch; i++)
		{
			comm_sendByte(msg->data[i]);
		}
	}
	msg->checksum = comm_calcChecksum(msg);
	comm_sendByte(msg->checksum >> 8);
	comm_sendByte(msg->checksum & 0xff);
}

/////////////////////////////////////////////////////////////////////
/// \brief comm_receivedMsg
/// \return
///		true if received new message in ISR

uint8_t comm_receivedMsg(void)
{
	if(comm_sm == NEWMESSAGE)
		return 1;
	else
		return 0;
}

/////////////////////////////////////////////////////////////////////
/// \brief comm_listen
/// \return
///		has to be called after a received message was processed and
///		the master can listen again

void comm_listen(void)
{
	comm_sm = WAITFORPACKAGE;
}

///////////////////////////////////////////////////////////////////////////
/// \brief comm_sendByte
///		sends byte to slave
/// \param byte

void comm_sendByte(uint8_t byte)
{
	// wait until data register is empty
	while( !(USART3->SR & 0x00000040) );
	USART_SendData(USART3, byte);
}
