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
#include "outf.h"
#include "xv11.h"
#include "slamdefs.h"
#include "slam.h"
#include "main.h"
#include "math.h"
#include "navigation_api.h"
#include "navigation.h"

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
	gui_clearAREA(element);

	LCD_Line(element->x,element->y + element->heigth, element->x + element->length, element->y + element->heigth, LCD_COLOR_BLACK);

	if(element->state == STAT_DROPPED)
	{
		for(u8 i = 2; i < STAT_STACK_SIZE + 1; i++)
		{
			UB_Font_DrawPString(element->x + 100, element->y + STAT_BATTERY_BORDER + 3 + i * 10, statusbar_readMessage(i), &pArial_10, LCD_COLOR_BLACK, statusbar_readMessageColor(i));
		}
	}

	UB_Font_DrawPString(element->x + 100, element->y + STAT_BATTERY_BORDER, statusbar_readMessage(1), &pArial_16, LCD_COLOR_BLACK, statusbar_readMessageColor(1));

	int8_t batt_percent = battery.percent;
	if(batt_percent > 100)
		batt_percent = 100;
	if(batt_percent < 0)
		batt_percent = 0;

	u16 batt_color_fill = LCD_COLOR_BRIGHTGREEN;
	u16 batt_color_border = LCD_COLOR_BLACK;
	if(batt_percent <= 25)
		batt_color_fill = LCD_COLOR_BRIGHTYELLOW;
	if(batt_percent <= 10)
		batt_color_fill = batt_color_border = LCD_COLOR_BRIGHTRED;

	LCD_Rectangle(element->x + STAT_BATTERY_BORDER, //Batterie Körper
				  element->y + STAT_BATTERY_BORDER,
				  element->x + STAT_BATTERY_BORDER + 35,
				  element->y + STAT_BATTERY_BORDER + 14,
				  batt_color_border, 0);

	LCD_Rectangle(element->x + STAT_BATTERY_BORDER + 35 + 1,
				  element->y + STAT_BATTERY_BORDER + 14 - 11,
				  element->x + STAT_BATTERY_BORDER + 35 + 2,
				  element->y + STAT_BATTERY_BORDER + 14 - 3,
				  batt_color_border, 0);

	LCD_Rectangle(element->x + STAT_BATTERY_BORDER + 1, //Batterie Füllung
				  element->y + STAT_BATTERY_BORDER + 1,
				  element->x + STAT_BATTERY_BORDER + (34 * batt_percent / 100), //1..34
				  element->y + STAT_BATTERY_BORDER + 14 - 1,
				  batt_color_fill, 1);

	char buffer[5];
	ltoa(buffer, batt_percent);
	if(batt_percent >= 100)
	{
		buffer[3] = '%';
		buffer[4] = 0;
	}
	else if(batt_percent >= 10)
	{
		buffer[2] = '%';
		buffer[3] = 0;
	}
	else
	{
		buffer[1] = '%';
		buffer[2] = 0;
	}


	UB_Font_DrawPString(element->x + STAT_BATTERY_BORDER + 44,
						element->y + STAT_BATTERY_BORDER,
						buffer, &pArial_16, LCD_COLOR_BLACK, LCD_COLOR_WHITE);
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
		float scale;
		if(MAP_SIZE_Y_MM > MAP_SIZE_X_MM) //Calculate scale of the actual map and the displayed map
			scale = ((MAP_SIZE_Y_MM / MAP_RESOLUTION_MM) / MAP_DRAW_HEIGHT);
		else
			scale = ((MAP_SIZE_X_MM / MAP_RESOLUTION_MM) / MAP_DRAW_WIDTH);

		LCD_SetClipRgn(element->x, element->y, element->x + element->length, element->y + element->heigth);
		LCD_Rectangle(element->x, element->y, element->x + element->length, element->y + element->heigth, LCD_COLOR_BLACK, 0);

		if(processedView)
			slam_LCD_DispMapProcessed(element->x + 1, element->y + 1, &slam); //Not nessesary to clear area (it is overwritten)
		else
			slam_LCD_DispMap(element->x + 1, element->y + 1, scale, &slam); //Not nessesary to clear area (it is overwritten)


		LCD_Line(element->x + (int)((slam.robot_pos.coord.x / MAP_RESOLUTION_MM) / scale),
				 element->y + (int)((MAP_SIZE_Y_MM - slam.robot_pos.coord.y) / MAP_RESOLUTION_MM / scale),
				 element->x + (int)((slam.robot_pos.coord.x / MAP_RESOLUTION_MM) / scale) + 10 * sinf((70 + slam.robot_pos.psi) * (M_PI / 180)),
				 element->y + (int)((MAP_SIZE_Y_MM - slam.robot_pos.coord.y) / MAP_RESOLUTION_MM / scale) - 10 * cosf((70 + slam.robot_pos.psi) * (M_PI / 180)),
				 LCD_COLOR_RED);
		LCD_Line(element->x + (int)((slam.robot_pos.coord.x / MAP_RESOLUTION_MM) / scale),
				 element->y + (int)((MAP_SIZE_Y_MM - slam.robot_pos.coord.y) / MAP_RESOLUTION_MM / scale),
				 element->x + (int)((slam.robot_pos.coord.x / MAP_RESOLUTION_MM) / scale) + 10 * sinf((110 + slam.robot_pos.psi) * (M_PI / 180)),
				 element->y + (int)((MAP_SIZE_Y_MM - slam.robot_pos.coord.y) / MAP_RESOLUTION_MM / scale) - 10 * cosf((110 + slam.robot_pos.psi) * (M_PI / 180)),
				 LCD_COLOR_RED);

		/*for(uint8_t i = 0; i <= MAP_SIZE_X_MM/200; i ++) //Scale
		{
			LCD_Line(element->x + 1 + (i * 20),
					 element->y + element->heigth,
					 element->x + 1 + (i * 20),
					 element->y + element->heigth - 4,
					 GUI_COLOR_FONT);
		}
		for(uint8_t i = 0; i <= MAP_SIZE_Y_MM/200; i ++)
		{
			LCD_Line(element->x + 202,
					 element->y + 1 + (i * 20),
					 element->x + 207,
					 element->y + 1 + (i * 20),
					 GUI_COLOR_FONT);
		}*/

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

		nav_waypoint_t *ptrWp = nav_wpStart;
		while(ptrWp != NULL) //Draw waypoint networw
		{
			int16_t line_x1, line_y1, line_x2, line_y2;

			line_x2 = element->x + ((ptrWp->x / MAP_RESOLUTION_MM) / scale);
			line_y2 = element->y + (((MAP_SIZE_Y_MM - ptrWp->y) / MAP_RESOLUTION_MM)) / scale;

			if(ptrWp->previous != NULL)
			{
				line_x1 = element->x + ((ptrWp->previous->x / MAP_RESOLUTION_MM) / scale);
				line_y1 = element->y + (((MAP_SIZE_Y_MM - ptrWp->previous->y) / MAP_RESOLUTION_MM) / scale);

				LCD_Line(line_x1, line_y1,
						 line_x2, line_y2, LCD_COLOR_BRIGHTBLUE);
			}

			LCD_Circle(line_x2, line_y2,
					   2, LCD_COLOR_BRIGHTRED, 0);

			ptrWp = ptrWp->next;
		}

		if(nextWp != NULL) //Goal defined
		{
			LCD_Line(element->x + (slam.robot_pos.coord.x / MAP_RESOLUTION_MM),
					 element->y + ((MAP_SIZE_Y_MM - slam.robot_pos.coord.y) / MAP_RESOLUTION_MM),
					 element->x + (nextWp->x / MAP_RESOLUTION_MM),
					 element->y + ((MAP_SIZE_Y_MM - nextWp->y) / MAP_RESOLUTION_MM),
					 LCD_COLOR_BRIGHTYELLOW);
		}

		LCD_SetClipRgn(0, 0, GetMaxX(), GetMaxY());
	}
}
