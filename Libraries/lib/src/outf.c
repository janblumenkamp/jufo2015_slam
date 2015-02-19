/**************************************************************************//*****
 * @file     printf.c
 * @brief    Implementation of several stdio.h methods, such as printf(),
 *           sprintf() and so on. This reduces the memory footprint of the
 *           binary when using those methods, compared to the libc implementation.
 *
 * VT100 Commands:
 *	\e[2J //clear
 *
 ********************************************************************************/
#include "stm32f4xx_conf.h"
#include <stdarg.h>
#include "outf.h"
#include "debug.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_conf.h"
#include "queue.h"

stream_t slamUI;
stream_t strlidar;
stream_t debug;
stream_t debugOS;
stream_t error;


//Puts a character directly to the usart (slow, but not basing on queues or interrupts)
int8_t usart2_put(char c)
{
	USART_SendData(USART2, (uint8_t) c);
	// Loop until the end of transmission
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);

	return c;
}

//Puts a character to the output queue, which (later) transmit its through the transmit isr. Fast, only for much data/debugging
int8_t usart2queue_put_spaceErrSent = 0;
int8_t usart2queue_put_NULLErrSent = 0;

int8_t usart2queue_put(char c)
{
	if(xQueueTXUSART2 != 0) //If queue was already created
	{
		usart2queue_put_NULLErrSent = 0;
		if(xQueueSendToBack(xQueueTXUSART2, &c, 0)) //character is in queue now
		{
			if(USART_GetITStatus (USART2 ,USART_IT_TXE) != SET) //If transmit interrupt is not active, activate it to send data from queue
				USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
		}
		else if(!usart2queue_put_spaceErrSent)
		{
			usart2queue_put_spaceErrSent = 1;
			foutf(&error, "txerr: Queue_SPACE");
		}
	}
	else if(!usart2queue_put_NULLErrSent)
	{
		usart2queue_put_NULLErrSent = 1;
		foutf(&error, "txerr: Queue_NULL");
	}

	if(usart2queue_put_NULLErrSent || usart2queue_put_spaceErrSent) //Instead, put character directly.
	{
		usart2_put(c);
	}
	/*
	if(xQueueSendToBack(xQueueTXUSART2, &c, 0)) //character is in queue now
	{
		if(USART_GetITStatus (USART2 ,USART_IT_TXE) != SET) //If transmit interrupt is not active, activate it to send data from queue
			USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
	}*/
	return c;
}

void out_init(void)
{
	/*
	 Foreground Colours
	30	Black
	31	Red
	32	Green
	33	Yellow
	34	Blue
	35	Magenta
	36	Cyan
	37	White

		Background Colours
	40	Black
	41	Red
	42	Green
	43	Yellow
	44	Blue
	45	Magenta
	46	Cyan
	47	White
	 */

	slamUI.active = 1;
	slamUI.bgcolor = 0;
	slamUI.textcolor = 0;
	slamUI.put_c = &usart2queue_put;

	debug.active = 1;
	debug.bgcolor = 0;//42; //green
	debug.textcolor = 0;//30; //black
	debug.put_c = &usart2queue_put;

	strlidar.active = 0;
	strlidar.bgcolor = 0;
	strlidar.textcolor = 0;
	strlidar.put_c = &usart2queue_put;

	debugOS.active = 1;
	debugOS.bgcolor = 0;//43; //Yellow
	debugOS.textcolor = 0;//30; //black
	debugOS.put_c = &usart2_put;

	error.active = 1;
	error.bgcolor = 0;//41; //red
	error.textcolor = 0;//30; //black
	error.put_c = &usart2_put;
}

void out_onOff(stream_t *stream, u_int8_t state)
{
	stream->active = state;
}

////////////////////////////////////////////////////////////////////////////////7
////////////////////////////////////////////////////////////////////////////////
/// Start of Library

/** Required for proper compilation. */
//struct _reent r = {0, (FILE *) 0, (FILE *) 1, (FILE *) 0};
//struct _reent *_impure_ptr = &r;

/**
 * @brief  Writes a character inside the given string. Returns 1.
 *
 * @param  pStr	Storage string.
 * @param  c    Character to write.
 */
signed int PutChar(char *pStr, char c)
{
	*pStr = c;
	return 1;
}


/**
 * @brief  Writes a string inside the given string.
 *
 * @param  pStr     Storage string.
 * @param  pSource  Source string.
 * @return  The size of the written
 */
signed int PutString(char *pStr, const char *pSource)
{
	signed int num = 0;

	while (*pSource != 0) {

		*pStr++ = *pSource++;
		num++;
	}

	return num;
}


/**
 * @brief  Writes an unsigned int inside the given string, using the provided fill &
 *         width parameters.
 *
 * @param  pStr  Storage string.
 * @param  fill  Fill character.
 * @param  width  Minimum integer width.
 * @param  value  Integer value.
 */
signed int PutUnsignedInt(
	char *pStr,
	char fill,
	signed int width,
	unsigned int value)
{
	signed int num = 0;

	/* Take current digit into account when calculating width */
	width--;

	/* Recursively write upper digits */
	if ((value / 10) > 0) {

		num = PutUnsignedInt(pStr, fill, width, value / 10);
		pStr += num;
	}

	/* Write filler characters */
	else {

		while (width > 0) {

			PutChar(pStr, fill);
			pStr++;
			num++;
			width--;
		}
	}

	/* Write lower digit */
	num += PutChar(pStr, (value % 10) + '0');

	return num;
}


/**
 * @brief  Writes a signed int inside the given string, using the provided fill & width
 *         parameters.
 *
 * @param pStr   Storage string.
 * @param fill   Fill character.
 * @param width  Minimum integer width.
 * @param value  Signed integer value.
 */
signed int PutSignedInt(
	char *pStr,
	char fill,
	signed int width,
	signed int value)
{
	signed int num = 0;
	unsigned int absolute;

	/* Compute absolute value */
	if (value < 0) {

		absolute = -value;
	}
	else {

		absolute = value;
	}

	/* Take current digit into account when calculating width */
	width--;

	/* Recursively write upper digits */
	if ((absolute / 10) > 0) {

		if (value < 0) {

			num = PutSignedInt(pStr, fill, width, -(absolute / 10));
		}
		else {

			num = PutSignedInt(pStr, fill, width, absolute / 10);
		}
		pStr += num;
	}
	else {

		/* Reserve space for sign */
		if (value < 0) {

			width--;
		}

		/* Write filler characters */
		while (width > 0) {

			PutChar(pStr, fill);
			pStr++;
			num++;
			width--;
		}

		/* Write sign */
		if (value < 0) {

			num += PutChar(pStr, '-');
			pStr++;
		}
	}

	/* Write lower digit */
	num += PutChar(pStr, (absolute % 10) + '0');

	return num;
}


/**
 * @brief  Writes an hexadecimal value into a string, using the given fill, width &
 *         capital parameters.
 *
 * @param pStr   Storage string.
 * @param fill   Fill character.
 * @param width  Minimum integer width.
 * @param maj    Indicates if the letters must be printed in lower- or upper-case.
 * @param value  Hexadecimal value.
 *
 * @return  The number of char written
 */
signed int PutHexa(
	char *pStr,
	char fill,
	signed int width,
	unsigned char maj,
	unsigned int value)
{
	signed int num = 0;

	/* Decrement width */
	width--;

	/* Recursively output upper digits */
	if ((value >> 4) > 0) {

		num += PutHexa(pStr, fill, width, maj, value >> 4);
		pStr += num;
	}
	/* Write filler chars */
	else {

		while (width > 0) {

			PutChar(pStr, fill);
			pStr++;
			num++;
			width--;
		}
	}

	/* Write current digit */
	if ((value & 0xF) < 10) {

		PutChar(pStr, (value & 0xF) + '0');
	}
	else if (maj) {

		PutChar(pStr, (value & 0xF) - 10 + 'A');
	}
	else {

		PutChar(pStr, (value & 0xF) - 10 + 'a');
	}
	num++;

	return num;
}



/* Global Functions ----------------------------------------------------------- */


/**
 * @brief  Stores the result of a formatted string into another string. Format
 *         arguments are given in a va_list instance.
 *
 * @param pStr    Destination string.
 * @param length  Length of Destination string.
 * @param pFormat Format string.
 * @param ap      Argument list.
 *
 * @return  The number of characters written.
 */
signed int vsnoutf(char *pStr, size_t length, const char *pFormat, va_list ap)
{
	char          fill;
	unsigned char width;
	signed int    num = 0;
	signed int    size = 0;

	/* Clear the string */
	if (pStr) {

		*pStr = 0;
	}

	/* Phase string */
	while (*pFormat != 0 && size < length) {

		/* Normal character */
		if (*pFormat != '%') {

			*pStr++ = *pFormat++;
			size++;
		}
		/* Escaped '%' */
		else if (*(pFormat+1) == '%') {

			*pStr++ = '%';
			pFormat += 2;
			size++;
		}
		/* Token delimiter */
		else {

			fill = ' ';
			width = 0;
			pFormat++;

			/* Parse filler */
			if (*pFormat == '0') {

				fill = '0';
				pFormat++;
			}

			/* Parse width */
			while ((*pFormat >= '0') && (*pFormat <= '9')) {

				width = (width*10) + *pFormat-'0';
				pFormat++;
			}

			/* Check if there is enough space */
			if (size + width > length) {

				width = length - size;
			}

			/* Parse type */
			switch (*pFormat) {
			case 'd':
			case 'i': num = PutSignedInt(pStr, fill, width, va_arg(ap, signed int)); break;
			case 'u': fill = '0'; num = PutUnsignedInt(pStr, fill, width, va_arg(ap, unsigned int)); break;
			case 'x': fill = '0'; num = PutHexa(pStr, fill, width, 0, va_arg(ap, unsigned int)); break;
			case 'X': fill = '0'; num = PutHexa(pStr, fill, width, 1, va_arg(ap, unsigned int)); break;
			case 's': num = PutString(pStr, va_arg(ap, char *)); break;
			case 'c': num = PutChar(pStr, va_arg(ap, unsigned int)); break;
			default:
				return -1;
			}

			pFormat++;
			pStr += num;
			size += num;
		}
	}

	/* NULL-terminated (final \0 is not counted) */
	if (size < length) {

		*pStr = 0;
	}
	else {

		*(--pStr) = 0;
		size--;
	}

	return size;
}


/**
 * @brief  Stores the result of a formatted string into another string. Format
 *         arguments are given in a va_list instance.
 *
 * @param pStr    Destination string.
 * @param length  Length of Destination string.
 * @param pFormat Format string.
 * @param ...     Other arguments
 *
 * @return  The number of characters written.
 */
signed int snoutf(char *pString, size_t length, const char *pFormat, ...)
{
	va_list    ap;
	signed int rc;

	va_start(ap, pFormat);
	rc = vsnoutf(pString, length, pFormat, ap);
	va_end(ap);

	return rc;
}


/**
 * @brief  Stores the result of a formatted string into another string. Format
 *         arguments are given in a va_list instance.
 *
 * @param pString  Destination string.
 * @param length   Length of Destination string.
 * @param pFormat  Format string.
 * @param ap       Argument list.
 *
 * @return  The number of characters written.
 */
signed int vsoutf(char *pString, const char *pFormat, va_list ap)
{
   return vsnoutf(pString, MAX_STRING_SIZE, pFormat, ap);
}

/**
 * @brief  Outputs a formatted string on the given stream. Format arguments are given
 *         in a va_list instance.
 *
 * @param pStream  Output stream.
 * @param pFormat  Format string
 * @param ap       Argument list.
 */
signed int vfoutf(stream_t *pStream, const char *pFormat, va_list ap)
{
	char pStr[MAX_STRING_SIZE];
	char pError[] = "stdio.c: increase MAX_STRING_SIZE\n";

	/* Write formatted string in buffer */
	if (vsoutf(pStr, pFormat, ap) >= MAX_STRING_SIZE) {

		out_fputs(pError, NULL);
		return -1;
	}
	else
	{
	/* Display string */
		return out_fputs(pStr, pStream);
	}
}


/**
 * @brief  Outputs a formatted string on the given stream, using a variable
 *         number of arguments.
 *
 * @param pStream  Output stream.
 * @param pFormat  Format string.
 */
signed int foutf(stream_t *pStream, const char *pFormat, ...)
{
	va_list ap;
	signed int result;

	/* Forward call to vfoutf */
	va_start(ap, pFormat);
	result = vfoutf(pStream, pFormat, ap);
	va_end(ap);

	return result;
}

/**
 * @brief  Writes a formatted string inside another string.
 *
 * @param pStr     torage string.
 * @param pFormat  Format string.
 */
signed int soutf(char *pStr, const char *pFormat, ...)
{
	va_list ap;
	signed int result;

	// Forward call to vsoutf
	va_start(ap, pFormat);
	result = vsoutf(pStr, pFormat, ap);
	va_end(ap);

	return result;
}


/**
 * @brief  Outputs a string with defined length on given stream.
 *
 * @param pStr  String to output.
 * @param len	lenght of string
 */

void out_puts_l(stream_t *pStream, const char *pStr, u_int32_t len)
{
	for(u_int32_t i = 0; i < len; i++)
	{
		if(pStream->put_c != NULL && pStream->active)
			pStream->put_c((char) pStr[i]);
	}
}

/**
 * @brief  Implementation of fputs using the DBGU as the standard output. Required
 *         for outf().
 *
 * @param pStr     String to write.
 * @param pStream  Output stream.
 *
 * @return  Number of characters written if successful, or -1 if the output
 *          stream is not stdout or stderr.
 */
signed int out_fputs(const char *pStr, stream_t *pStream) {

	signed int num = 0;

	if(pStream->active)
	{
		char vt100[7] = "\e[30m"; //textcolor black

		if(pStream->bgcolor != 0)
		{
			vt100[3] = pStream->bgcolor % 10 + 48; //10. 48: ASCII 0
			vt100[2] = pStream->bgcolor / 10 + 48; //1
			out_puts_l(pStream, vt100, 6);
		}

		if(pStream->textcolor != 0)
		{
			vt100[3] = pStream->textcolor % 10 + 48; //10. 48: ASCII 0
			vt100[2] = pStream->textcolor / 10 + 48; //1
			out_puts_l(pStream, vt100, 6);
		}

		while (*pStr != 0)
		{
			if(pStream->put_c != NULL)
			{
				if((pStream->put_c((char)*pStr) == -1))
					return -1;
			}
			else
				out_n_fputc('?');

			num++;
			pStr++;
		}
	}

	return num;
}



/**
 * @brief  Called if stream pointer is NULL
 *
 * @param c        Character to write.
 * @param pStream  Output stream.
 * @param The character written if successful, or -1 if the output stream is
 *        not stdout or stderr.
 */
signed int out_n_fputc(signed int c)
{
	USART_SendData(USART2, (uint8_t) c);
	// Loop until the end of transmission
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);

	return c;
}

/* --------------------------------- End Of File ------------------------------ */
