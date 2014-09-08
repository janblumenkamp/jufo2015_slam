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


typedef enum {
	COMM_SYSTEMSTATUS, //Can be: 0 (ok). Bit: 1: error. Bit2: Motor off.
	COMM_DIST_BACK_RIGHT_LSB, COMM_DIST_BACK_RIGHT_MSB,
	COMM_DIST_RIGHT_BACK_LSB, COMM_DIST_RIGHT_BACK_MSB,
	COMM_DIST_LEFT_BACK_LSB, COMM_DIST_LEFT_BACK_MSB,
	COMM_DIST_BACK_LEFT_LSB, COMM_DIST_BACK_LEFT_MSB,
	COMM_DIST_FRONT_FRONT_LSB, COMM_DIST_FRONT_FRONT_MSB,
	COMM_DIST_BACK_BACK_LSB, COMM_DIST_BACK_BACK_MSB,
	COMM_ADC6_LSB, COMM_ADC6_MSB,
	COMM_DIST_DOWN_LSB, COMM_DIST_DOWN_MSB,
	COMM_ADC8_LSB, COMM_ADC8_MSB,
	COMM_ADC_BATTERY_RAW_LSB, COMM_ADC_BATTERY_RAW_MSB,
	COMM_DIST_FRONT_LEFT_LSB, COMM_DIST_FRONT_LEFT_MSB,
	COMM_DIST_LEFT_FRONT_LSB, COMM_DIST_LEFT_FRONT_MSB,
	COMM_SENS_IMPASSE_1_LSB, COMM_SENS_IMPASSE_1_MSB,
	COMM_SENS_IMPASSE_2_LSB, COMM_SENS_IMPASSE_2_MSB,
	COMM_DIST_RIGHT_FRONT_LSB, COMM_DIST_RIGHT_FRONT_MSB,
	COMM_DIST_FRONT_RIGHT_LSB, COMM_DIST_FRONT_RIGHT_MSB,
	COMM_BATTERY_MV_LSB, COMM_BATTERY_MV_MSB,
	COMM_BATTERY_PERCENT,
	COMM_MOT_ENC_L_LSB_0, COMM_MOT_ENC_L_1, COMM_MOT_ENC_L_2, COMM_MOT_ENC_L_MSB_3,
	COMM_MOT_ENC_R_LSB_0, COMM_MOT_ENC_R_1, COMM_MOT_ENC_R_2, COMM_MOT_ENC_R_MSB_3,
	COMM_MOT_SPEED_L_IS,
	COMM_MOT_SPEED_R_IS,
	COMM_MOT_SPEED_L_TO,
	COMM_MOT_SPEED_R_TO,
	COMM_MOT_DRIVER_STANDBY,
	COMM_LED_MODE, //Can be: 0 (control by sub), 1 (control by main -> Hue, saturation and value)
	COMM_LED_HUE,
	COMM_LED_SAT,
	COMM_LED_VAL,
	/////// DO NOT ADD ANYTHING BELOW THIS LINE!!!//////////////
	COMM_REGISTERS_CNT
} REGISTERS;

extern comm_msg_t receivedMessage;
extern uint8_t messageBuffer[COMM_BUFSIZE];
extern comm_msg_t sendMessage;

//Initialisation of the comm interface
extern void comm_init(void);

//calculates checksum of the given message
extern uint16_t comm_calcChecksum(comm_msg_t *msg);

//Sends given package
extern void comm_sendPackage(comm_msg_t *msg);

//Sends given package, listens for answer and manages it
uint8_t comm_bidirectionalPackage(comm_msg_t *msg, uint8_t max_tries);

// returns true if received new message in ISR
extern uint8_t comm_receivedMsg(void);

//has to be called after a received message was processed and the master can listen again
extern void comm_listen(void);

//Sends byte to slave
extern void comm_sendByte(uint8_t byte);

#endif // COMM_API_H
