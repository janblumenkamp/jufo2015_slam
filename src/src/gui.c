//////////////////////////////////////////////////////////////////////////////////////
/// gui.c - Design and configuration of the gui, gui task of RTOS and managing of
/// event queries and menu/page visualisation.
///
/// ADDING NEW ELEMENTS: see gui.h
//////////////////////////////////////////////////////////////////////////////////////

#include "FreeRTOS.h"
#include "task.h"

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include "utils.h"
#include "debug.h"
#include "printf.h"
#include "xv11.h"
#include "main.h"

#include "stm32_ub_touch_ADS7843.h"
#include "SSD1963_api.h"
#include "SSD1963.h"
#include "stm32_ub_font.h"
#include "gui.h"
#include "gui_graphics.h"
#include "gui_areaElements.h"

GUI_ELEMENT gui_element[GUI_ELEMENTS_CNT]; //GUI Elements structure

#define STATUSBAR_HEIGHT 25 //Definition/constants of AREAS
#define STATUSBAR_LENGTH GetMaxX()

#define PAGE_GRID_DIST 5 //Distance in pixels between GUI elements

u8 menu = 0; //active menu/page (menu statemachine)

//////////////////////////////////////////////////////////////////////////////
/// Private Prototypes (for a detailed description see each function)

void gui_el_pages_putInvisible(void);

//Events
void gui_el_event_area_statusbar(ELEMENT_EVENT *event);
void gui_el_event_mbtn_map(ELEMENT_EVENT *event);
void gui_el_event_sw_startMapping(ELEMENT_EVENT *event);
void gui_el_event_sw_showScan(ELEMENT_EVENT *event);
void gui_el_event_mbtn_view(ELEMENT_EVENT *event);
void gui_el_event_mbtn_settings(ELEMENT_EVENT *event);
void gui_el_event_sw_lidar(ELEMENT_EVENT *event);
void gui_el_event_btn_caltouch(ELEMENT_EVENT *event);
void gui_el_event_btn_reset(ELEMENT_EVENT *event);

///////////////////////////////////////////////////////////////////////////////
/// \brief gui_el_pages_putInvisible
/// The elements state has to be set to GUI_EL_INVISIBLE here, if the element
///	is a element of a page (if it is a sub-element of one of the MBTN Buttons).
///	This is important to disable the event functions of the not-active elements
///	(if the page is changed or the statusbar is dropped, all elements are
///	are firstly set as inactive and then in the menu statemachine corresponding
///	init state of the new page resetted to the active state).
///////////////////////////////////////////////////////////////////////////////

void gui_el_pages_putInvisible(void)
{
	//Map
	gui_element[GUI_EL_SW_STARTMAPPING].state = GUI_EL_INVISIBLE;
	gui_element[GUI_EL_SW_SHOWSCAN].state = GUI_EL_INVISIBLE;
	gui_element[GUI_EL_AREA_MAP].state = GUI_EL_INVISIBLE;

	//View

	//Settings
	gui_element[GUI_EL_SW_LIDAR].state = GUI_EL_INVISIBLE;
	gui_element[GUI_EL_BTN_CALTOUCH].state = GUI_EL_INVISIBLE;
	gui_element[GUI_EL_BTN_RESET].state = GUI_EL_INVISIBLE;
}

/////////////////////////////////////////////////////////////////////////////
//////////////////////                     //////////////////////////////////
//////////////////////   EVENT FUNCTIONS   //////////////////////////////////
//////////////////////                     //////////////////////////////////
/////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
/// \brief gui_el_event_area_statusbar
/// \param event

void gui_el_event_area_statusbar(ELEMENT_EVENT *event)
{
	if(event->released)
	{
		if(gui_element[GUI_EL_AREA_STATUSBAR_TOP].state == STAT_NOT_DROPPED)
		{
			gui_element[GUI_EL_AREA_STATUSBAR_TOP].state = STAT_DROPPED;

			gui_el_pages_putInvisible();

			gui_clearAREA(&gui_element[GUI_EL_AREA_STATUSBAR_DROPPED]);

			gui_drawAREAstatusbar(&gui_element[GUI_EL_AREA_STATUSBAR_TOP]);
		}
		else
		{
			gui_element[GUI_EL_AREA_STATUSBAR_TOP].state = STAT_NOT_DROPPED;

			gui_drawAREAstatusbar(&gui_element[GUI_EL_AREA_STATUSBAR_TOP]);
			gui_clearAREA(&gui_element[GUI_EL_AREA_STATUSBAR_DROPPED]);
			gui_drawMBTN(&gui_element[GUI_EL_MBTN_MAP]);
			gui_drawMBTN(&gui_element[GUI_EL_MBTN_VIEW]);
			gui_drawMBTN(&gui_element[GUI_EL_MBTN_SETTINGS]);

			menu --; //Draw active menu again (go into active menu init state)
		}
	}
}

/////////////////////////////////////////////////////////////////////////////
/// \brief gui_el_event_mbtn_map
/// \param event

void gui_el_event_mbtn_map(ELEMENT_EVENT *event)
{
	if(event->released)
	{
		gui_element[GUI_EL_MBTN_MAP].state = MBTN_ACTIVE;
		gui_element[GUI_EL_MBTN_VIEW].state = MBTN_NOT_ACTIVE;
		gui_element[GUI_EL_MBTN_SETTINGS].state = MBTN_NOT_ACTIVE;

		gui_el_pages_putInvisible();

		gui_clearAREA(&gui_element[GUI_EL_AREA_CONTENT]);
		//MBTN Map is already redrawn in gui handler (because it was touched)
		gui_drawMBTN(&gui_element[GUI_EL_MBTN_VIEW]); //Redraw menubuttons because their state may has changed
		gui_drawMBTN(&gui_element[GUI_EL_MBTN_SETTINGS]);

		menu = MENU_MAP_INIT;
	}
}

/////////////////////////////////////////////////////////////////////////////
/// \brief gui_el_event_sw_startMapping
/// \param event

void gui_el_event_sw_startMapping(ELEMENT_EVENT *event)
{
	if(event->released)
	{
		if(gui_element[GUI_EL_SW_STARTMAPPING].state == SW_OFF)
		{
			//Set flags...
			gui_element[GUI_EL_SW_STARTMAPPING].state = SW_ON;
		}
		else
		{
			//Unset flags...
			gui_element[GUI_EL_SW_STARTMAPPING].state = SW_OFF;
		}
	}
}

////////////////////////////////////////////////////////////////////////////
/// \brief gui_el_event_sw_showScan
/// \param event

void gui_el_event_sw_showScan(ELEMENT_EVENT *event)
{
	if(event->released)
	{
		if(gui_element[GUI_EL_SW_SHOWSCAN].state == SW_OFF)
		{
			//Set flags...
			gui_element[GUI_EL_SW_SHOWSCAN].state = SW_ON;
		}
		else
		{
			//Unset flags...
			gui_element[GUI_EL_SW_SHOWSCAN].state = SW_OFF;
		}
	}
}

/////////////////////////////////////////////////////////////////////////////
/// \brief gui_el_event_mbtn_view
/// \param event

void gui_el_event_mbtn_view(ELEMENT_EVENT *event)
{
	if(event->released)
	{
		gui_element[GUI_EL_MBTN_VIEW].state = MBTN_ACTIVE;
		gui_element[GUI_EL_MBTN_MAP].state = MBTN_NOT_ACTIVE;
		gui_element[GUI_EL_MBTN_SETTINGS].state = MBTN_NOT_ACTIVE;

		gui_el_pages_putInvisible();

		gui_clearAREA(&gui_element[GUI_EL_AREA_CONTENT]);
		gui_drawMBTN(&gui_element[GUI_EL_MBTN_MAP]); //Redraw menubuttons because their state may has changed
		//MBTN View is already redrawn in gui handler (because it was touched)
		gui_drawMBTN(&gui_element[GUI_EL_MBTN_SETTINGS]);

		menu = MENU_VIEW_INIT;
	}
}

/////////////////////////////////////////////////////////////////////////////
/// \brief gui_el_event_mbtn_settings
/// \param event

void gui_el_event_mbtn_settings(ELEMENT_EVENT *event)
{
	if(event->released)
	{
		gui_element[GUI_EL_MBTN_SETTINGS].state = MBTN_ACTIVE;
		gui_element[GUI_EL_MBTN_MAP].state = MBTN_NOT_ACTIVE;
		gui_element[GUI_EL_MBTN_VIEW].state = MBTN_NOT_ACTIVE;

		gui_el_pages_putInvisible();

		gui_clearAREA(&gui_element[GUI_EL_AREA_CONTENT]);
		gui_drawMBTN(&gui_element[GUI_EL_MBTN_MAP]); //Redraw menubuttons because their state may has changed
		gui_drawMBTN(&gui_element[GUI_EL_MBTN_VIEW]);
		//MBTN Settings is already redrawn in gui handler (because it was touched)

		menu = MENU_SETTINGS_INIT;
	}
}

/////////////////////////////////////////////////////////////////////////////
/// \brief gui_el_event_sw_lidar
/// \param event

void gui_el_event_sw_lidar(ELEMENT_EVENT *event)
{
	if(event->released)
	{
		if(gui_element[GUI_EL_SW_LIDAR].state == SW_OFF)
		{
			xv11_state(XV11_STARTING);
			gui_element[GUI_EL_SW_LIDAR].state = SW_BUSY;
		}
		else
		{
			xv11_state(XV11_OFF);
			gui_element[GUI_EL_SW_LIDAR].state = SW_OFF;
		}
	}
}

/////////////////////////////////////////////////////////////////////////////
/// \brief gui_el_event_btn_caltouch
/// \param event

void gui_el_event_btn_caltouch(ELEMENT_EVENT *event)
{
	if(event->pressed)
	{
		gui_element[GUI_EL_BTN_CALTOUCH].state = BTN_ACTIVE;
	}
	if(event->released)
	{
		gui_element[GUI_EL_BTN_CALTOUCH].state = BTN_NOT_ACTIVE;
		menu = MENU_CALIBRATION;
	}
}

/////////////////////////////////////////////////////////////////////////////
/// \brief gui_el_event_btn_reset
/// \param event

void gui_el_event_btn_reset(ELEMENT_EVENT *event)
{
	if(event->clicked)
	{
		gui_element[GUI_EL_BTN_RESET].state = BTN_ACTIVE;
	}
	if(event->doubleclick)
	{
		gui_element[GUI_EL_BTN_RESET].state = BTN_NOT_ACTIVE;
		NVIC_SystemReset();
	}
}

/////////////////////////////////////////////////////////////////////////////
/// \brief gui_init
/// Initialisation of the GUI. For every element you have to select the
///	x/y position and the length. Corresponding to the selected ID you also
///	have to select the other given variables like the font, the label etc.
///	If existing, don’t forget to add the event function in GUI_ELEMENT.event
///	(pointer to the function)
/////////////////////////////////////////////////////////////////////////////

void gui_init(void)
{
	//Init the elements ids
	gui_element[GUI_EL_AREA_STATUSBAR_TOP].id = EL_ID_AREA;
	gui_element[GUI_EL_AREA_STATUSBAR_DROPPED].id = EL_ID_AREA;
	gui_element[GUI_EL_AREA_CONTENT].id = EL_ID_AREA;
	gui_element[GUI_EL_MBTN_MAP].id = EL_ID_MBTN;
		gui_element[GUI_EL_AREA_MAP].id = EL_ID_AREA;
		gui_element[GUI_EL_SW_STARTMAPPING].id = EL_ID_SW;
		gui_element[GUI_EL_SW_SHOWSCAN].id = EL_ID_SW;
		//gui_element[GUI_EL_SLI_MAP_SCALE].id = EL_ID_SLI;
	gui_element[GUI_EL_MBTN_VIEW].id = EL_ID_MBTN;
	gui_element[GUI_EL_MBTN_SETTINGS].id = EL_ID_MBTN;
		gui_element[GUI_EL_BTN_CALTOUCH].id = EL_ID_BTN;
		gui_element[GUI_EL_BTN_RESET].id = EL_ID_BTN;
		gui_element[GUI_EL_SW_LIDAR].id = EL_ID_SW;

	//Load standard settings
	graphics_init(gui_element);

	//Individual settings
	gui_element[GUI_EL_AREA_STATUSBAR_TOP].length = STATUSBAR_LENGTH;
	gui_element[GUI_EL_AREA_STATUSBAR_TOP].heigth = STATUSBAR_HEIGHT;
	gui_element[GUI_EL_AREA_STATUSBAR_TOP].action = &gui_el_event_area_statusbar;
	gui_element[GUI_EL_AREA_STATUSBAR_TOP].state = STAT_NOT_DROPPED;

	gui_element[GUI_EL_AREA_STATUSBAR_DROPPED].length = STATUSBAR_LENGTH;
	gui_element[GUI_EL_AREA_STATUSBAR_DROPPED].y = gui_element[GUI_EL_AREA_STATUSBAR_TOP].heigth + 1;
	gui_element[GUI_EL_AREA_STATUSBAR_DROPPED].heigth = GetMaxY() - gui_element[GUI_EL_AREA_STATUSBAR_DROPPED].y;
	gui_element[GUI_EL_AREA_STATUSBAR_DROPPED].state = GUI_EL_INTOUCHABLE;

	gui_element[GUI_EL_MBTN_MAP].x = -(MBTN_STD_HEIGHT / 2);
	gui_element[GUI_EL_MBTN_MAP].length += (MBTN_STD_HEIGHT / 2) ;
	gui_element[GUI_EL_MBTN_MAP].y = gui_element[GUI_EL_AREA_STATUSBAR_TOP].y + gui_element[GUI_EL_AREA_STATUSBAR_TOP].heigth + 1;
	gui_element[GUI_EL_MBTN_MAP].label = (char *)"Map/Scan";
	gui_element[GUI_EL_MBTN_MAP].action = &gui_el_event_mbtn_map;
	gui_element[GUI_EL_MBTN_MAP].state = MBTN_ACTIVE;

		gui_element[GUI_EL_SW_STARTMAPPING].label = (char *)"Mapping:";
		gui_element[GUI_EL_SW_STARTMAPPING].action = &gui_el_event_sw_startMapping;
		gui_element[GUI_EL_SW_STARTMAPPING].x = PAGE_GRID_DIST;
		gui_element[GUI_EL_SW_STARTMAPPING].y = gui_element[GUI_EL_MBTN_MAP].y + gui_element[GUI_EL_MBTN_MAP].heigth + PAGE_GRID_DIST;
		gui_element[GUI_EL_SW_STARTMAPPING].state = SW_OFF;

		gui_element[GUI_EL_SW_SHOWSCAN].label = (char *)"Show scan:";
		gui_element[GUI_EL_SW_SHOWSCAN].action = &gui_el_event_sw_showScan;
		gui_element[GUI_EL_SW_SHOWSCAN].x = PAGE_GRID_DIST;
		gui_element[GUI_EL_SW_SHOWSCAN].y = gui_element[GUI_EL_SW_STARTMAPPING].y + gui_element[GUI_EL_SW_STARTMAPPING].heigth + PAGE_GRID_DIST;
		gui_element[GUI_EL_SW_SHOWSCAN].state = SW_OFF;

		gui_element[GUI_EL_AREA_MAP].x = gui_element[GUI_EL_SW_STARTMAPPING].x + gui_element[GUI_EL_SW_STARTMAPPING].length + PAGE_GRID_DIST;
		gui_element[GUI_EL_AREA_MAP].y = gui_element[GUI_EL_SW_STARTMAPPING].y;
		gui_element[GUI_EL_AREA_MAP].length = GetMaxX() - gui_element[GUI_EL_AREA_MAP].x;
		gui_element[GUI_EL_AREA_MAP].heigth = GetMaxY() - gui_element[GUI_EL_AREA_MAP].y;
		gui_element[GUI_EL_AREA_MAP].state = GUI_EL_INTOUCHABLE;

	gui_element[GUI_EL_AREA_CONTENT].length = GetMaxX();
	gui_element[GUI_EL_AREA_CONTENT].y = gui_element[GUI_EL_MBTN_MAP].y + gui_element[GUI_EL_MBTN_MAP].heigth + 1;
	gui_element[GUI_EL_AREA_CONTENT].heigth = GetMaxY() - gui_element[GUI_EL_AREA_CONTENT].y;
	gui_element[GUI_EL_AREA_CONTENT].state = GUI_EL_INTOUCHABLE;

	gui_element[GUI_EL_MBTN_VIEW].x = gui_element[GUI_EL_MBTN_MAP].x + gui_element[GUI_EL_MBTN_MAP].length + 1;
	gui_element[GUI_EL_MBTN_VIEW].y = gui_element[GUI_EL_MBTN_MAP].y;
	gui_element[GUI_EL_MBTN_VIEW].label = (char *)"Info";
	gui_element[GUI_EL_MBTN_VIEW].action = &gui_el_event_mbtn_view;
	gui_element[GUI_EL_MBTN_VIEW].state = MBTN_NOT_ACTIVE;

	gui_element[GUI_EL_MBTN_SETTINGS].x = gui_element[GUI_EL_MBTN_VIEW].x + gui_element[GUI_EL_MBTN_VIEW].length + 1;
	gui_element[GUI_EL_MBTN_SETTINGS].y = gui_element[GUI_EL_MBTN_MAP].y;
	gui_element[GUI_EL_MBTN_SETTINGS].label = (char *)"Settings";
	gui_element[GUI_EL_MBTN_SETTINGS].action = &gui_el_event_mbtn_settings;
	gui_element[GUI_EL_MBTN_SETTINGS].state = MBTN_NOT_ACTIVE;

		gui_element[GUI_EL_BTN_CALTOUCH].x = PAGE_GRID_DIST;
		gui_element[GUI_EL_BTN_CALTOUCH].y = gui_element[GUI_EL_MBTN_MAP].y + gui_element[GUI_EL_MBTN_MAP].heigth + PAGE_GRID_DIST;
		gui_element[GUI_EL_BTN_CALTOUCH].label = (char *)"Calibrate touchscreen";
		gui_element[GUI_EL_BTN_CALTOUCH].action = &gui_el_event_btn_caltouch;
		gui_element[GUI_EL_BTN_CALTOUCH].state = GUI_EL_INVISIBLE;

		gui_element[GUI_EL_BTN_RESET].x = gui_element[GUI_EL_BTN_CALTOUCH].x + gui_element[GUI_EL_BTN_CALTOUCH].length + PAGE_GRID_DIST;
		gui_element[GUI_EL_BTN_RESET].y = gui_element[GUI_EL_BTN_CALTOUCH].y;
		gui_element[GUI_EL_BTN_RESET].length /= 2;
		gui_element[GUI_EL_BTN_RESET].label = (char *)"RESET";
		gui_element[GUI_EL_BTN_RESET].action = &gui_el_event_btn_reset;
		gui_element[GUI_EL_BTN_RESET].state = GUI_EL_INVISIBLE;

		gui_element[GUI_EL_SW_LIDAR].x = PAGE_GRID_DIST;
		gui_element[GUI_EL_SW_LIDAR].y = gui_element[GUI_EL_BTN_CALTOUCH].y + gui_element[GUI_EL_BTN_CALTOUCH].heigth + 5;
		gui_element[GUI_EL_SW_LIDAR].label = (char *)"Lidar:";
		gui_element[GUI_EL_SW_LIDAR].action = &gui_el_event_sw_lidar;
		gui_element[GUI_EL_SW_LIDAR].state = GUI_EL_INVISIBLE;

	for(u8 i = 0; i < STAT_STACK_SIZE; i++)
		statusbar_addMessage((char *) "-", LCD_COLOR_WHITE);

	statusbar_addMessage((char *) "Jugend Forscht 2015 - System started.", LCD_COLOR_WHITE);

	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO); //Calibrate Touchscreen with this Button
}

portTASK_FUNCTION( vGUITask, pvParameters )
{
	portTickType xLastWakeTime;

	#if(configDEBUG_MESSAGES == 1)
		printf("xTask GUI started.\r\n");
	#endif

	xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
		//printf("Switch height: %i\n Switch length: %i\n\n", element[i].heigth, element[i].length);

		gui_handler(gui_element);

		if(STM_EVAL_PBGetState(BUTTON_USER))
		{
			menu = MENU_CALIBRATION;
		}

		switch (menu) {
		case MENU_INIT:
			LCD_Fill(LCD_COLOR_WHITE);
			gui_drawAREAstatusbar(&gui_element[GUI_EL_AREA_STATUSBAR_TOP]);
			gui_drawMBTN(&gui_element[GUI_EL_MBTN_MAP]);
			gui_drawMBTN(&gui_element[GUI_EL_MBTN_VIEW]);
			gui_drawMBTN(&gui_element[GUI_EL_MBTN_SETTINGS]);

			menu = MENU_INIT_IDLE;

		case MENU_INIT_IDLE: //Wait for events...

			if(gui_element[GUI_EL_MBTN_MAP].state == MBTN_ACTIVE)
				menu = MENU_MAP_INIT;
			else if(gui_element[GUI_EL_MBTN_VIEW].state == MBTN_ACTIVE)
				menu = MENU_VIEW_INIT;
			else if(gui_element[GUI_EL_MBTN_SETTINGS].state == MBTN_ACTIVE)
				menu = MENU_SETTINGS_INIT;

			break;

		case MENU_MAP_INIT:

			//Area Map
			//SW Startmapping
			// //Map_Scale

			gui_element[GUI_EL_AREA_MAP].state = GUI_EL_INTOUCHABLE;
			gui_element[GUI_EL_SW_STARTMAPPING].state = SW_OFF;
			gui_element[GUI_EL_SW_SHOWSCAN].state = SW_OFF;

			gui_drawSW(&gui_element[GUI_EL_SW_STARTMAPPING]);
			gui_drawSW(&gui_element[GUI_EL_SW_SHOWSCAN]);

			menu = MENU_MAP_IDLE;

		case MENU_MAP_IDLE:

			gui_drawAREAmap(&gui_element[GUI_EL_AREA_MAP]);
			
			break; //Map idle (waiting for touch events)

		case MENU_VIEW_INIT:
			//Draw View
			menu = MENU_VIEW_IDLE;

		case MENU_VIEW_IDLE:
			break; //View idle (waiting for touch events)
		case MENU_SETTINGS_INIT: //Settings active

			switch (xv11_state(XV11_GETSTATE)) {
			case XV11_OFF:		gui_element[GUI_EL_SW_LIDAR].state = SW_OFF;	break;
			case XV11_STARTING:	gui_element[GUI_EL_SW_LIDAR].state = SW_BUSY;	break;
			case XV11_ON:		gui_element[GUI_EL_SW_LIDAR].state = SW_ON;		break;
			default:															break;
			}
			gui_drawSW(&gui_element[GUI_EL_SW_LIDAR]);

			gui_element[GUI_EL_BTN_CALTOUCH].state = BTN_NOT_ACTIVE;
			gui_drawBTN(&gui_element[GUI_EL_BTN_CALTOUCH]);

			gui_element[GUI_EL_BTN_RESET].state = BTN_NOT_ACTIVE;
			gui_drawBTN(&gui_element[GUI_EL_BTN_RESET]);

			menu = MENU_SETTINGS_IDLE;

		case MENU_SETTINGS_IDLE:

			if(gui_element[GUI_EL_SW_LIDAR].state != GUI_EL_INVISIBLE)
			{
				if(xv11_state(XV11_GETSTATE) == XV11_STARTING)
				{
					if(gui_element[GUI_EL_SW_LIDAR].state != SW_BUSY)
					{
						gui_element[GUI_EL_SW_LIDAR].state = SW_BUSY;
						gui_drawSW(&gui_element[GUI_EL_SW_LIDAR]);
					}
				}
				else if(xv11_state(XV11_GETSTATE) == XV11_ON)
				{
					if(gui_element[GUI_EL_SW_LIDAR].state != SW_ON)
					{
						gui_element[GUI_EL_SW_LIDAR].state = SW_ON;
						gui_drawSW(&gui_element[GUI_EL_SW_LIDAR]);
					}
				}
			}
			break; //Settings idle (waiting for touch events)

		case MENU_CALIBRATION: //Calibration mode:
			if(UB_Touch_Calibrate() == 1)
			{
				menu = MENU_INIT;
			}
			break;

		default: menu = MENU_INIT;
			break;
		}


		//LCD_Fill(LCD_COLOR_WHITE);
		//UB_Font_DrawPString(10,10,"TestStringABC123",&pArial_16,LCD_COLOR_BLACK,LCD_COLOR_WHITE);

		/*int16_t xv11_max = 0;
		for(u16 i = 0; i < 360; i++)
		{
			if(xv11.dist_polar[i] > xv11_max)
				xv11_max = xv11.dist_polar[i];
		}

		//printf("max x: %i; max y: %i, max polar: %i\n", xv11.dist_cartesian_max.x, xv11.dist_cartesian_max.y, xv11.dist_polar_max);
		for(u16 i = 0; i < 360; i++)
		{
			//printf("x: %i; y: %i\n", xv11.dist_cartesian[10].x, xv11.dist_cartesian[10].y);
			if(xv11.dist_polar[i] > 0)
			{
				int16_t lcd_x = (xv11.dist_cartesian[i].x/(xv11.dist_polar_max/((DISP_VER_RESOLUTION/2)-20))) + (DISP_HOR_RESOLUTION/2);
				int16_t lcd_y = (xv11.dist_cartesian[i].y/(xv11.dist_polar_max/((DISP_VER_RESOLUTION/2)-20))) + (DISP_VER_RESOLUTION/2);
				LCD_Line(DISP_HOR_RESOLUTION/2, DISP_VER_RESOLUTION/2, lcd_x, lcd_y, LCD_COLOR_CYAN);
				//LCD_PutPixel(xv11.dist_cartesian[i].x + DISP_HOR_RESOLUTION/2, xv11.dist_cartesian[i].y + DISP_VER_RESOLUTION/2, LCD_COLOR_BRIGHTRED);
			}
		}*/

		vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_RATE_MS ) );
	}
}
