//--------------------------------------------------------------
// File     : stm32_ub_font.c
// Datum    : 24.07.2014
// Version  : 1.5
// Autor    : UB
// EMail    : mc-4u(@)t-online.de
// Web      : www.mikrocontroller-4u.de 
// CPU      : STM32F4
// IDE      : CooCox CoIDE 1.7.4
// GCC      : 4.7 2012q4
// Module   : STM32_UB_LCD_ST7783
// Funktion : Text-LCD Funktionen
//--------------------------------------------------------------


//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "stm32_ub_font.h"
#include "SSD1963.h"
#include "math.h"


//--------------------------------------------------------------
// Zeichnet ein Ascii-Zeichen eines monospaced Fonts an x,y Position
// mit Vorder- und Hintergrundfarbe (Font = max 16 Pixel breite)
// -> Font muss mit &-Operator uebergeben werden
//--------------------------------------------------------------
void UB_Font_DrawChar(uint16_t x, uint16_t y, uint8_t ascii, UB_Font *font, uint16_t vg, uint16_t bg)
{
	uint16_t xn,yn,start_maske,maske;
	const uint16_t *wert;

	ascii -= 32;
	wert=&font->table[ascii * font->height];

	start_maske=0x80;
	if(font->width>8) start_maske=0x8000;

	for(yn = 0; yn < font->height; yn++)
	{
		maske=start_maske;
		// Cursor setzen
		//UB_LCD_SetCursor2Draw(x,yn+y);
		for(xn = 0; xn < font->width; xn++)
		{
			if((wert[yn] & maske) == 0x00)
			{
				// Pixel in Hintergrundfarbe zeichnen
				LCD_PutPixel(x + xn, yn + y, bg);
			}
			else
			{
				// Pixel in Vordergrundfarbe zeichnen
				LCD_PutPixel(x + xn, yn + y, vg);
			}
			maske=(maske>>1);
		}
	}
}


//--------------------------------------------------------------
// Zeichnet einen String eines monospaced Fonts an x,y Position
// mit Vorder- und Hintergrundfarbe (Font = max 16 Pixel breite)
// -> Font muss mit &-Operator uebergeben werden
//--------------------------------------------------------------
void UB_Font_DrawString(uint16_t x, uint16_t y,char *ptr, UB_Font *font, uint16_t vg, uint16_t bg)
{
	uint16_t pos;

	pos=x;
	while (*ptr != 0)
	{
		UB_Font_DrawChar(pos,y,*ptr,font,vg,bg);
		pos+=font->width;
		ptr++;
	}
}


//--------------------------------------------------------------
// Zeichnet ein Ascii-Zeichen eines monospaced Fonts an x,y Position
// mit Vorder- und Hintergrundfarbe (Font = max 32 Pixel breite)
// -> Font muss mit &-Operator uebergeben werden
//--------------------------------------------------------------
void UB_Font_DrawChar32(uint16_t x, uint16_t y, uint8_t ascii, UB_Font32 *font, uint16_t vg, uint16_t bg)
{
	uint16_t xn,yn;
	uint32_t start_maske,maske;
	const uint32_t *wert;


	ascii -= 32;
	wert=&font->table[ascii * font->height];

	start_maske=0x80;
	if(font->width>8) start_maske=0x8000;
	if(font->width>16) start_maske=0x80000000;

	for(yn = 0; yn < font->height; yn++)
	{
		maske=start_maske;
		// Cursor setzen
		for(xn = 0; xn < font->width; xn++)
		{
			if((wert[yn] & maske) == 0x00)
			{
				// Pixel in Hintergrundfarbe zeichnen
				LCD_PutPixel(x + xn, yn + y, bg);
			}
			else
			{
				// Pixel in Vordergrundfarbe zeichnen
				LCD_PutPixel(x + xn, yn + y, vg);
			}
			maske=(maske>>1);
		}
	}
}


//--------------------------------------------------------------
// Zeichnet einen String eines monospaced Fonts an x,y Position
// mit Vorder- und Hintergrundfarbe (Font = max 32 Pixel breite)
// -> Font muss mit &-Operator uebergeben werden
//--------------------------------------------------------------
void UB_Font_DrawString32(uint16_t x, uint16_t y,char *ptr, UB_Font32 *font, uint16_t vg, uint16_t bg)
{
	uint16_t pos;

	pos=x;
	while (*ptr != 0)
	{
		UB_Font_DrawChar32(pos,y,*ptr,font,vg,bg);
		pos+=font->width;
		ptr++;
	}
}


//--------------------------------------------------------------
// Zeichnet ein Ascii-Zeichen eines proportional Fonts an x,y Position
// mit Vorder- und Hintergrundfarbe (Font = max 16 Pixel breite)
// -> Font muss mit &-Operator uebergeben werden
// return : breite des gezeichneten Zeichens
//--------------------------------------------------------------
uint16_t UB_Font_DrawPChar(uint16_t x, uint16_t y, uint8_t ascii, UB_pFont *font, uint16_t vg, uint16_t bg)
{
	uint16_t xn,yn,start_maske,maske,width;
	const uint16_t *wert;

	// check auf Grenze
	if(ascii<font->first_char) return 0;
	if(ascii>font->last_char) return 0;

	ascii -= font->first_char;
	wert=&font->table[ascii * (font->height+1)];
	width=wert[0];

	start_maske=0x01;
	start_maske=start_maske<<(width-1);

	for(yn = 0; yn < font->height; yn++)
	{
		maske=start_maske;
		// Cursor setzen
		for(xn = 0; xn < width; xn++)
		{
			if((wert[yn+1] & maske) == 0x00)
			{
				// Pixel in Hintergrundfarbe zeichnen
				LCD_PutPixel(x + xn, yn + y, bg);
			}
			else
			{
				// Pixel in Vordergrundfarbe zeichnen
				LCD_PutPixel(x + xn, yn + y, vg);
			}
			maske=(maske>>1);
		}
	}

	return(width);
}


//--------------------------------------------------------------
// Zeichnet einen String eines proportional Fonts an x,y Position
// mit Vorder- und Hintergrundfarbe (Font = max 16 Pixel breite)
// -> Font muss mit &-Operator uebergeben werden
//--------------------------------------------------------------
void UB_Font_DrawPString(uint16_t x, uint16_t y,char *ptr, UB_pFont *font, uint16_t vg, uint16_t bg)
{
	uint16_t pos,width;

	pos=x;
	while (*ptr != 0)
	{
		width=UB_Font_DrawPChar(pos,y,*ptr,font,vg,bg);
		pos+=width;
		ptr++;
	}
}


//--------------------------------------------------------------
// Zeichnet ein Ascii-Zeichen eines proportional Fonts an x,y Position
// mit Vorder- und Hintergrundfarbe (Font = max 32 Pixel breite)
// -> Font muss mit &-Operator uebergeben werden
// return : breite des gezeichneten Zeichens
//--------------------------------------------------------------
uint16_t UB_Font_DrawPChar32(uint16_t x, uint16_t y, uint8_t ascii, UB_pFont32 *font, uint16_t vg, uint16_t bg)
{
	uint16_t xn,yn,width;
	uint32_t start_maske,maske;
	const uint32_t *wert;

	// check auf Grenze
	if(ascii<font->first_char) return 0;
	if(ascii>font->last_char) return 0;

	ascii -= font->first_char;
	wert=&font->table[ascii * (font->height+1)];
	width=wert[0];

	start_maske=0x01;
	start_maske=start_maske<<(width-1);

	for(yn = 0; yn < font->height; yn++)
	{
		maske=start_maske;
		// Cursor setzen
		for(xn = 0; xn < width; xn++)
		{
			if((wert[yn+1] & maske) == 0x00)
			{
				// Pixel in Hintergrundfarbe zeichnen
				LCD_PutPixel(x + xn, yn + y, bg);
			}
			else
			{
				// Pixel in Vordergrundfarbe zeichnen
				LCD_PutPixel(x + xn, yn + y, vg);
			}
			maske=(maske>>1);
		}
	}

	return(width);
}


//--------------------------------------------------------------
// Zeichnet einen String eines proportional Fonts an x,y Position
// mit Vorder- und Hintergrundfarbe (Font = max 32 Pixel breite)
// -> Font muss mit &-Operator uebergeben werden
//--------------------------------------------------------------
void UB_Font_DrawPString32(uint16_t x, uint16_t y,char *ptr, UB_pFont32 *font, uint16_t vg, uint16_t bg)
{
	uint16_t pos,width;

	pos=x;
	while (*ptr != 0)
	{
		width=UB_Font_DrawPChar32(pos,y,*ptr,font,vg,bg);
		pos+=width;
		ptr++;
	}
}

//--------------------------------------------------------------------
void ltoa(char * buf, long val)
{
	unsigned char i;
	char neg = 0;
	char tmp_buf[21];

	if(val<0) {
		val = -val;
		neg = 1;
	}

	for(i = 0; val > 0; val /= 10)
		tmp_buf[i++] = (val % 10) + '0';

	if(i == 0)
		tmp_buf[i++] = '0';

	if(neg)
		tmp_buf[i++] = '-';

	while(i>0)
		*buf++ = tmp_buf[--i];
}
