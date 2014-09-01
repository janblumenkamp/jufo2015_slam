#include "gui_graphics.h"
#include "gui_areaElements.h"
#include "SSD1963_api.h"
#include "SSD1963.h"
#include "stm32_ub_font.h"
#include "stm32_ub_touch_ADS7843.h"
#include "printf.h"

GUI_ELEMENT_STAT_STACK stat_stack;

void statusbar_addMessage(char *message, u16 color_bg)
{
	stat_stack.message[stat_stack.index] = message;
	stat_stack.color_bg[stat_stack.index] = color_bg;
	stat_stack.index ++;
	if(stat_stack.index == STAT_STACK_SIZE)
		stat_stack.index = 0;
}

char *statusbar_readMessage(int8_t index)
{
	index = stat_stack.index - index;
	while(index < 0)
		index += STAT_STACK_SIZE;

	return stat_stack.message[index];
}

u16 statusbar_readMessageColor(int8_t index)
{
	index = stat_stack.index - index;
	while(index < 0)
		index += STAT_STACK_SIZE;

	return stat_stack.color_bg[index];
}

#define STAT_BATTERY_BORDER 5 //Pixel

void gui_drawAREAstatusbar(GUI_ELEMENT *element)
{
	LCD_Line(element->x,element->y + element->heigth, element->x + element->length, element->y + element->heigth, LCD_COLOR_BLACK);

	if(element->state == STAT_DROPPED)
	{
		for(u8 i = 2; i < STAT_STACK_SIZE + 1; i++)
		{
			UB_Font_DrawPString(element->x + 100, element->y + STAT_BATTERY_BORDER + 3 + i * 10, statusbar_readMessage(i), &pArial_10, LCD_COLOR_BLACK, statusbar_readMessageColor(i));
		}
	}

	UB_Font_DrawPString(element->x + 100, element->y + STAT_BATTERY_BORDER, statusbar_readMessage(1), &pArial_16, LCD_COLOR_BLACK, statusbar_readMessageColor(1));

	u8 batt_percent = 60;
	u16 batt_color = LCD_COLOR_BRIGHTGREEN;
	if(batt_percent <= 20)
		batt_color = LCD_COLOR_BRIGHTYELLOW;
	if(batt_percent <= 5)
		batt_color = LCD_COLOR_BRIGHTRED;

	LCD_Rectangle(element->x + STAT_BATTERY_BORDER, //Batterie Körper
				  element->y + STAT_BATTERY_BORDER,
				  element->x + STAT_BATTERY_BORDER + 35,
				  element->y + STAT_BATTERY_BORDER + 14,
				  LCD_COLOR_BLACK, 0);

	LCD_Rectangle(element->x + STAT_BATTERY_BORDER + 35 + 1,
				  element->y + STAT_BATTERY_BORDER + 14 - 11,
				  element->x + STAT_BATTERY_BORDER + 35 + 2,
				  element->y + STAT_BATTERY_BORDER + 14 - 3,
				  LCD_COLOR_BLACK, 0);

	LCD_Rectangle(element->x + STAT_BATTERY_BORDER + 1, //Batterie Füllung
				  element->y + STAT_BATTERY_BORDER + 1,
				  element->x + STAT_BATTERY_BORDER + (34 / (100 / batt_percent)),
				  element->y + STAT_BATTERY_BORDER + 14 - 1,
				  batt_color, 1);
}

void gui_drawAREAscan(GUI_ELEMENT *element)
{
	gui_clearAREA(element);
	LCD_Line(element->x,
			 element->y,
			 element->x,
			 element->y + element->heigth, GUI_COLOR_FONT);
}
