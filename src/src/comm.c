//////////////////////////////////////////////////////////////////////////////////////
/// comm.c - Interface to the Subcontroller
///
///
////////////////////////////////////////////////////////////////////////////////

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include "main.h"
#include "comm.h"
#include "comm_api.h"

static volatile uint8_t comm_reg[COMM_REGSIZE];

//////////////////////////////////////////////////////////////////////
/// \brief comm_handler
///		handles the queries to the slave (to call as often as possible)

uint8_t comm_handler_sm = 0;

void comm_handler(void)
{
	if(comm_receivedMsg()) //new package arrived!
	{
		//Process...
		if(comm_calcChecksum(&receivedMessage) == receivedMessage.checksum) //Checksum matches, if write access write registers. Sens answer.
		{
			sendMessage.reg = 255;
			if(receivedMessage.batch_write) //Master wants to write into slave
			{
				for(uint8_t i = 0; i < receivedMessage.batch; i++)
				{
					comm_reg[receivedMessage.reg + i] = receivedMessage.data[i];
				}
				sendMessage.batch_write = 0;
				sendMessage.batch = 0;
			}
			else //Master wants to read -> Slave writes to master
			{
				sendMessage.batch_write = 1;
				sendMessage.batch = receivedMessage.batch;
				sendMessage.data = (uint8_t *) &comm_reg[receivedMessage.reg];
			}
		}
		else //Send error message.
		{
			sendMessage.reg = 254;
			sendMessage.batch_write = 0;
			sendMessage.batch = 0;
		}

		comm_sendPackage(&sendMessage);

		comm_listen();
	}
}
