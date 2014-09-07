////////////////////////////////////////////////////////////////////////////////
/// gui_areaElements.c - Elementar functions etc. of the GUI Element area!
/// Areas can be used for everything, but how they look has to be defined here.
///
/// ADDING NEW ELEMENTS: see gui.h
//////////////////////////////////////////////////////////////////////////////////////

#include "gui_graphics.h"
#include "gui_areaElements.h"
#include "SSD1963_api.h"
#include "SSD1963.h"
#include "stm32_ub_font.h"
#include "stm32_ub_touch_ADS7843.h"
#include "printf.h"
#include "xv11.h"
#include "slamdefs.h"
#include "slam.h"
#include "main.h"
#include "math.h"

//Stack of the statusbar history
GUI_ELEMENT_STAT_STACK stat_stack;

//Statusbar: Distance in Pixel between battery symbol and display border
#define STAT_BATTERY_BORDER 5 //Pixel

//////////////////////////////////////////////////////////////////////////////
/// Private Prototypes (for a detailed description see each function)

char *statusbar_readMessage(int8_t index);
u16 statusbar_readMessageColor(int8_t index);

///////////////////////////////////////////////////////////////////////////////
/// \brief statusbar_addMessage
/// \param message
/// \param color_bg
/// Adds a new message to the stack of the statusbar

void statusbar_addMessage(char *message, u16 color_bg)
{
	stat_stack.message[stat_stack.index] = message;
	stat_stack.color_bg[stat_stack.index] = color_bg;
	stat_stack.index ++;
	if(stat_stack.index == STAT_STACK_SIZE)
		stat_stack.index = 0;
}

/////////////////////////////////////////////////////////////////////////////
/// \brief statusbar_readMessage
/// \param index
/// \return
/// Returns any message of the statusbar history (1 is the newest message, 0
/// the oldest)

char *statusbar_readMessage(int8_t index)
{
	index = stat_stack.index - index;
	while(index < 0)
		index += STAT_STACK_SIZE;

	return stat_stack.message[index];
}

/////////////////////////////////////////////////////////////////////////////
/// \brief statusbar_readMessageColor
/// \param index
/// \return
/// Returns any color of any message of the statusbar history (1 is the color
/// of the newest message, 0 of the oldest)

u16 statusbar_readMessageColor(int8_t index)
{
	index = stat_stack.index - index;
	while(index < 0)
		index += STAT_STACK_SIZE;

	return stat_stack.color_bg[index];
}

///////////////////////////////////////////////////////////////////////////
/// \brief gui_drawAREAstatusbar
/// \param element
/// Draws Statusbar

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

/////////////////////////////////////////////////////////////////
/// \brief gui_drawAREAmap
/// \param element
/// Draws SLAM Map

u8 show_scan = 1; //Display the Lidar scan in the map or don’t?

void gui_drawAREAmap(GUI_ELEMENT *element)
{
	if(element->state != GUI_EL_INVISIBLE)
	{
		LCD_SetClipRgn(element->x, element->y, element->x + element->length, element->y + element->heigth);
		LCD_Rectangle(element->x, element->y, element->x + element->length, element->y + element->heigth, LCD_COLOR_BLACK, 0);

		slam_LCD_DispMap(element->x + 1, element->y + 1, &slam); //Not nessesary to clear area (it is overwritten)


		LCD_Line(element->x + (slam.robot_pos.coord.x / MAP_RESOLUTION_MM),
				 element->y + (slam.robot_pos.coord.y / MAP_RESOLUTION_MM),
				 element->x + (slam.robot_pos.coord.x / MAP_RESOLUTION_MM) + 10 * sinf((290 + slam.robot_pos.psi) * (M_PI / 180)),
				 element->y + (slam.robot_pos.coord.y / MAP_RESOLUTION_MM) + 10 * cosf((290 + slam.robot_pos.psi) * (M_PI / 180)),
				 LCD_COLOR_RED);
		LCD_Line(element->x + (slam.robot_pos.coord.x / MAP_RESOLUTION_MM),
				 element->y + (slam.robot_pos.coord.y / MAP_RESOLUTION_MM),
				 element->x + (slam.robot_pos.coord.x / MAP_RESOLUTION_MM) + 10 * sinf((250 + slam.robot_pos.psi) * (M_PI / 180)),
				 element->y + (slam.robot_pos.coord.y / MAP_RESOLUTION_MM) + 10 * cosf((250 + slam.robot_pos.psi) * (M_PI / 180)),
				 LCD_COLOR_RED);
		/*LCD_Line(element->x,
				 element->y + element->heigth,
				 element->x + element->length - 40,
				 element->y + element->heigth,
				 GUI_COLOR_FONT);

		LCD_Line(element->x + element->length,
				 element->y,
				 element->x + element->length,
				 element->y + element->heigth - 40,
				 GUI_COLOR_FONT);*/

		//u16 mapval = 0;


		if(show_scan)
		{
			for(u16 i = 0; i < 360; i++)
			{
				/*if(xv11.dist_polar[i] > 0)
				{
					int16_t lcd_x = (xv11.dist_cartesian[i].x/(xv11.dist_polar_max/((DISP_VER_RESOLUTION/2)-20))) + (DISP_HOR_RESOLUTION/2);
					int16_t lcd_y = (xv11.dist_cartesian[i].y/(xv11.dist_polar_max/((DISP_VER_RESOLUTION/2)-20))) + (DISP_VER_RESOLUTION/2);
					LCD_Line(DISP_HOR_RESOLUTION/2, DISP_VER_RESOLUTION/2, lcd_x, lcd_y, LCD_COLOR_CYAN);
					//LCD_PutPixel(xv11.dist_cartesian[i].x + DISP_HOR_RESOLUTION/2, xv11.dist_cartesian[i].y + DISP_VER_RESOLUTION/2, LCD_COLOR_BRIGHTRED);
				}*/
				/*if(xv11.dist_polar[i] > 0)
				{
					int16_t lcd_x = (xv11.dist_cartesian[i].x/(xv11.dist_polar_max/((element->heigth/2)-20))) + (element->length/2);
					int16_t lcd_y = (xv11.dist_cartesian[i].y/(xv11.dist_polar_max/((element->heigth/2)-20))) + (element->heigth/2);
					LCD_Line(element->x + (element->length/2), element->y + (element->heigth/2), lcd_x, lcd_y, LCD_COLOR_BLUE);
					//LCD_PutPixel(xv11.dist_cartesian[i].x + DISP_HOR_RESOLUTION/2, xv11.dist_cartesian[i].y + DISP_VER_RESOLUTION/2, LCD_COLOR_BRIGHTRED);
				}*/
			}
		}

		LCD_SetClipRgn(0, 0, GetMaxX(), GetMaxY());
	}
}
