#ifndef PRINTF_H
#define PRINTF_H

#include <stdio.h>
#include "stm32f4xx_conf.h"
#include <stdarg.h>

extern signed int vsnprintf(char *pStr, size_t length, const char *pFormat, va_list ap);

extern signed int snprintf(char *pString, size_t length, const char *pFormat, ...);

extern signed int vsprintf(char *pString, const char *pFormat, va_list ap);

extern signed int vfprintf(FILE *pStream, const char *pFormat, va_list ap);

extern signed int vprintf(const char *pFormat, va_list ap);

extern signed int fprintf(FILE *pStream, const char *pFormat, ...);

extern signed int printf(const char *pFormat, ...);

extern signed int sprintf(char *pStr, const char *pFormat, ...);

extern signed int puts(const char *pStr);

extern signed int fputc(signed int c, FILE *pStream);

extern signed int fputs(const char *pStr, FILE *pStream);

#endif // PRINTF_H
