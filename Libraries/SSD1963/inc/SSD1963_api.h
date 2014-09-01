#ifndef SSD1963_API_H
#define SSD1963_API_H

#include "stm32f4xx.h"

/*********************************************************************
* Macros: RGB565CONVERT(red, green, blue)
*
* Overview: Converts true color into 5:6:5 RGB format.
*
* PreCondition: none
*
* Input: Red, Green, Blue components.
*
* Output: 5 bits red, 6 bits green, 5 bits blue color.
*
* Side Effects: none
*
********************************************************************/
	#define RGB565CONVERT(red, green, blue) (u16) (((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3))


/*********************************************************************
* Overview: Some basic colors definitions.
*********************************************************************/
#define LCD_COLOR_BLACK               RGB565CONVERT(0,    0,      0)
#define LCD_COLOR_BRIGHTBLUE          RGB565CONVERT(0,    0,      255)
#define LCD_COLOR_BRIGHTGREEN         RGB565CONVERT(0,    255,    0)
#define LCD_COLOR_BRIGHTCYAN          RGB565CONVERT(0,    255,    255)
#define LCD_COLOR_BRIGHTRED           RGB565CONVERT(255,  0,      0)
#define LCD_COLOR_BRIGHTMAGENTA       RGB565CONVERT(255,  0,      255)
#define LCD_COLOR_BRIGHTYELLOW        RGB565CONVERT(255,  255,    0)
#define LCD_COLOR_BLUE                RGB565CONVERT(0,    0,      128)
#define LCD_COLOR_GREEN               RGB565CONVERT(0,    128,    0)
#define LCD_COLOR_CYAN                RGB565CONVERT(0,    128,    128)
#define LCD_COLOR_RED                 RGB565CONVERT(128,  0,      0)
#define LCD_COLOR_MAGENTA             RGB565CONVERT(128,  0,      128)
#define LCD_COLOR_BROWN               RGB565CONVERT(255,  128,    0)
#define LCD_COLOR_LIGHTGRAY           RGB565CONVERT(128,  128,    128)
#define LCD_COLOR_DARKGRAY            RGB565CONVERT(64,   64,     64)
#define LCD_COLOR_LIGHTBLUE           RGB565CONVERT(128,  128,    255)
#define LCD_COLOR_LIGHTGREEN          RGB565CONVERT(128,  255,    128)
#define LCD_COLOR_LIGHTCYAN           RGB565CONVERT(128,  255,    255)
#define LCD_COLOR_LIGHTRED            RGB565CONVERT(255,  128,    128)
#define LCD_COLOR_LIGHTMAGENTA        RGB565CONVERT(255,  128,    255)
#define LCD_COLOR_YELLOW              RGB565CONVERT(255,  255,    128)
#define LCD_COLOR_WHITE               RGB565CONVERT(255,  255,    255)

#define LCD_COLOR_GRAY0       	    RGB565CONVERT(224,  224,    224)
#define LCD_COLOR_GRAY1         	    RGB565CONVERT(192,  192,    192)
#define LCD_COLOR_GRAY2               RGB565CONVERT(160,  160,    160)
#define LCD_COLOR_GRAY3               RGB565CONVERT(128,  128,    128)
#define LCD_COLOR_GRAY4               RGB565CONVERT(96,   96,     96)
#define LCD_COLOR_GRAY5               RGB565CONVERT(64,   64,     64)
#define LCD_COLOR_GRAY6	            RGB565CONVERT(32,   32,     32)

u16 Lcd_Color565(u32 RGB);  // RGB颜色转为16位(565)
typedef union
{
  u16 U16;
  u8 U8[2];
}ColorTypeDef;

//void LCD_Text(u16 x, u16 y, u8 *str, u16 len,u16 Color, u16 bkColor);
void LCD_Line(int16_t x0, int16_t y0, int16_t x1, int16_t y1,int16_t color);
void LCD_Fill(u16 color);
void LCD_Circle(int16_t cx,int16_t cy,int16_t r,u16 color,u8 fill);
void LCD_Rectangle(int16_t left, int16_t top, int16_t right, int16_t bottom, u16 color, u8 fill);
void LCD_Square(int16_t x0, int16_t y0, int16_t width, u16 color,u8 fill);
void LCD_ClearCharBox(u16 x,u16 y,u16 color);

void LCD_DispPic_FullSize(const unsigned char *str);
void LCD_DispPic(u16 x0, u16 y0, const unsigned char *str);

//int power (int m, int n);

#endif
