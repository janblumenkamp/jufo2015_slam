#ifndef OUTF_H
#define OUTF_H

#include "stm32f4xx_conf.h"
#include <stdarg.h>
#include "main.h"

// Maximum string size allocation (in bytes)
#define MAX_STRING_SIZE     512

typedef struct {
	u8 active; //Stream active or not?
	u8 textcolor; //VT100 terminal command colors
	u8 bgcolor;
	int8_t (*put_c)(char c); //Called if next char of given stream is processed
} stream_t;

extern stream_t slamUI;
extern stream_t strlidar;
extern stream_t debug;
extern stream_t debugOS;
extern stream_t error;

extern void out_init(void);

extern void out_onOff(stream_t *stream, u_int8_t state);

extern signed int vsnoutf(char *pStr, size_t length, const char *pFormat, va_list ap);

extern signed int snoutf(char *pString, size_t length, const char *pFormat, ...);

extern signed int vsoutf(char *pString, const char *pFormat, va_list ap);

extern signed int vfoutf(stream_t *pStream, const char *pFormat, va_list ap);

extern signed int foutf(stream_t *pStream, const char *pFormat, ...);

extern signed int soutf(char *pStr, const char *pFormat, ...);

extern void out_puts_l(stream_t *pStream, const char *pStr, u_int32_t len);

extern signed int out_n_fputc(signed int c);

extern signed int out_fputs(const char *pStr, stream_t *pStream);

#endif // OUTF_H
