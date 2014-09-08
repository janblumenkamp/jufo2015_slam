/////////////////////////////////////////////////////////////////////////////////
/// Communication with slave - API (core functions)
/////////////////////////////////////////////////////////////////////////////////

#ifndef COMM_API_H
#define COMM_API_H

#define UART_COMM_BAUD_RATE    115200

#define COMM_BUFSIZE 128 //In Bytes. Buffer for read/write access

#define COMM_BATCH_WRITE	0x80 //Batch write bit
#define COMM_BATCH			0x7F //Batch length bits

//Message/Package struct
typedef struct {
	uint8_t reg; //Desired register
	uint8_t batch:7; //Batchlength
	uint8_t batch_write:1; //Write access?
	uint8_t *data; //Pointer to buffer
	uint16_t checksum;
} comm_msg_t;

typedef enum {
	WAITFORPACKAGE,
	GET_REGISTER, GET_BATCH, GET_DATA, GET_CHK_LSB, GET_CHK_MSB,
	NEWMESSAGE
} COMM_SM;

extern comm_msg_t receivedMessage;
extern uint8_t messageBuffer[COMM_BUFSIZE];
extern comm_msg_t sendMessage;

//Initialisation of the comm interface
extern void comm_init(void);

//calculates checksum of the given message
extern uint16_t comm_calcChecksum(comm_msg_t *msg);

//Sends given package
extern void comm_sendPackage(comm_msg_t *msg);

// returns true if received new message in ISR
extern uint8_t comm_receivedMsg(void);

//has to be called after a received message was processed and the master can listen again
extern void comm_listen(void);

//Sends byte to slave
extern void comm_sendByte(uint8_t byte);

#endif // COMM_API_H
