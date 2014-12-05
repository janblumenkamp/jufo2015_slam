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
#include "outf.h"
#include "xv11.h"
#include "main.h"

#include "stm32_ub_touch_ADS7843.h"
#include "SSD1963_api.h"
#include "SSD1963.h"
#include "stm32_ub_font.h"
#include "gui.h"
#include "gui_graphics.h"
#include "gui_areaElements.h"
#include "slam.h"
#include "navigation_api.h"
#include "navigation.h"

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
void gui_el_event_sw_processedview(ELEMENT_EVENT *event);
void gui_el_event_btn_clearMap(ELEMENT_EVENT *event);
void gui_el_event_btn_setWp(ELEMENT_EVENT *event);
void gui_el_event_mbtn_view(ELEMENT_EVENT *event);
void gui_el_event_mbtn_settings(ELEMENT_EVENT *event);
void gui_el_event_sw_lidar(ELEMENT_EVENT *event);
void gui_el_event_sw_strdebug(ELEMENT_EVENT *event);
void gui_el_event_sw_strdebugos(ELEMENT_EVENT *event);
void gui_el_event_sw_strerr(ELEMENT_EVENT *event);
void gui_el_event_sw_strslamui(ELEMENT_EVENT *event);
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
	gui_element[GUI_EL_SW_PROCESSEDVIEW].state = GUI_EL_INVISIBLE;
	gui_element[GUI_EL_AREA_MAP].state = GUI_EL_INVISIBLE;
	gui_element[GUI_EL_BTN_CLEARMAP].state = GUI_EL_INVISIBLE;
	gui_element[GUI_EL_BTN_SETWP].state = GUI_EL_INVISIBLE;

	//Info

	//Settings
	gui_element[GUI_EL_SW_LIDAR].state = GUI_EL_INVISIBLE;
	gui_element[GUI_EL_BTN_CALTOUCH].state = GUI_EL_INVISIBLE;
	gui_element[GUI_EL_BTN_RESET].state = GUI_EL_INVISIBLE;
	gui_element[GUI_EL_SW_STRDEBUG].state = GUI_EL_INVISIBLE;
	gui_element[GUI_EL_SW_STRDEBUGOS].state = GUI_EL_INVISIBLE;
	gui_element[GUI_EL_SW_STRERR].state = GUI_EL_INVISIBLE;
	gui_element[GUI_EL_SW_STRSLAMUI].state = GUI_EL_INVISIBLE;

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
			mapping = 1;
			gui_element[GUI_EL_SW_STARTMAPPING].state = SW_ON;
		}
		else
		{
			mapping = 0;
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
			show_scan = 1;
			gui_element[GUI_EL_SW_SHOWSCAN].state = SW_ON;
		}
		else
		{
			show_scan = 0;
			gui_element[GUI_EL_SW_SHOWSCAN].state = SW_OFF;
		}
	}
}

////////////////////////////////////////////////////////////////////////////
/// \brief gui_el_event_sw_processedview
/// \param event

void gui_el_event_sw_processedview(ELEMENT_EVENT *event)
{
	if(event->released)
	{
		if(gui_element[GUI_EL_SW_PROCESSEDVIEW].state == SW_OFF)
		{
			processedView = 1;
			gui_element[GUI_EL_SW_PROCESSEDVIEW].state = SW_ON;
		}
		else
		{
			processedView = 0;
			gui_element[GUI_EL_SW_PROCESSEDVIEW].state = SW_OFF;
		}
	}
}

////////////////////////////////////////////////////////////////////////////
/// \brief gui_el_event_btn_clearMap
/// \param event

void gui_el_event_btn_clearMap(ELEMENT_EVENT *event)
{
	if(event->released)
	{
		for(u8 z = 0; z < MAP_SIZE_Z_LAYERS; z ++)
			for(u16 y = 0; y < (MAP_SIZE_Y_MM/MAP_RESOLUTION_MM); y++)
				for(u16 x = 0; x < (MAP_SIZE_X_MM / MAP_RESOLUTION_MM); x ++)
					slam.map.px[x][y][z] = 127;

		nav_initWaypointStack(); //clear waypoint list
		nextWp = NULL;
	}
}

////////////////////////////////////////////////////////////////////////////
/// \brief gui_el_event_btn_setWp
/// \param event

void gui_el_event_btn_setWp(ELEMENT_EVENT *event)
{
	if(event->released)
	{
		if(setWaypoints)
		{
			gui_element[GUI_EL_BTN_SETWP].state = BTN_NOT_ACTIVE;
			setWaypoints = 0;
		}
		else
		{
			gui_element[GUI_EL_BTN_SETWP].state = BTN_ACTIVE;
			setWaypoints = 1;
		}
	}
}

////////////////////////////////////////////////////////////////////////////
/// \brief gui_el_event_area_map
/// \param event

void gui_el_event_area_map(ELEMENT_EVENT *event)
{
	if(event->doubleclick)
	{
		slam.robot_pos.psi += 45;
		if(slam.robot_pos.psi > 359)
			slam.robot_pos.psi = 0;
	}
	else if(event->clicked)
	{
		if(setWaypoints)
		{
			nav_waypoint_t wp;

			wp.x = (Touch_Data.pos.xp - gui_element[GUI_EL_AREA_MAP].x) * MAP_RESOLUTION_MM;
			wp.y = MAP_SIZE_Y_MM - (Touch_Data.pos.yp - gui_element[GUI_EL_AREA_MAP].y) * MAP_RESOLUTION_MM;

			nav_attachWaypoint(&wp);
		}
		else
		{
			slam.robot_pos.coord.x = (Touch_Data.pos.xp - gui_element[GUI_EL_AREA_MAP].x) * MAP_RESOLUTION_MM;
			slam.robot_pos.coord.y = MAP_SIZE_Y_MM - (Touch_Data.pos.yp - gui_element[GUI_EL_AREA_MAP].y) * MAP_RESOLUTION_MM;
		}
	}

	//foutf(debug, "new robot position x: %i, y: %i, psi: %i\n", slam.robot_pos.coord.x, slam.robot_pos.coord.y, slam.robot_pos.psi);
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
/// \brief gui_el_event_sw_strdebug
/// \param event

void gui_el_event_sw_strdebug(ELEMENT_EVENT *event)
{
	if(event->released)
	{
		if(gui_element[GUI_EL_SW_STRDEBUG].state == SW_OFF)
		{
			debug.active = 1;
			gui_element[GUI_EL_SW_STRDEBUG].state = SW_ON;
		}
		else
		{
			debug.active = 0;
			gui_element[GUI_EL_SW_STRDEBUG].state = SW_OFF;
		}
	}
}

/////////////////////////////////////////////////////////////////////////////
/// \brief gui_el_event_sw_strdebugos
/// \param event

void gui_el_event_sw_strdebugos(ELEMENT_EVENT *event)
{
	if(event->released)
	{
		if(gui_element[GUI_EL_SW_STRDEBUGOS].state == SW_OFF)
		{
			debugOS.active = 1;
			gui_element[GUI_EL_SW_STRDEBUGOS].state = SW_ON;
		}
		else
		{
			debugOS.active = 0;
			gui_element[GUI_EL_SW_STRDEBUGOS].state = SW_OFF;
		}
	}
}

/////////////////////////////////////////////////////////////////////////////
/// \brief gui_el_event_sw_strerr
/// \param event

void gui_el_event_sw_strerr(ELEMENT_EVENT *event)
{
	if(event->released)
	{
		if(gui_element[GUI_EL_SW_STRERR].state == SW_OFF)
		{
			error.active = 1;
			gui_element[GUI_EL_SW_STRERR].state = SW_ON;
		}
		else
		{
			error.active = 0;
			gui_element[GUI_EL_SW_STRERR].state = SW_OFF;
		}
	}
}

/////////////////////////////////////////////////////////////////////////////
/// \brief gui_el_event_sw_strslamui
/// \param event

void gui_el_event_sw_strslamui(ELEMENT_EVENT *event)
{
	if(event->released)
	{
		if(gui_element[GUI_EL_SW_STRSLAMUI].state == SW_OFF)
		{
			out_onOff(&slamUI, 1);
			gui_element[GUI_EL_SW_STRSLAMUI].state = SW_ON;
		}
		else
		{
			out_onOff(&slamUI, 0);
			gui_element[GUI_EL_SW_STRSLAMUI].state = SW_OFF;
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
		gui_drawBTN(&gui_element[GUI_EL_BTN_CALTOUCH]);
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
		gui_element[GUI_EL_SW_PROCESSEDVIEW].id = EL_ID_SW;
		//gui_element[GUI_EL_SLI_MAP_SCALE].id = EL_ID_SLI;
		gui_element[GUI_EL_BTN_CLEARMAP].id = EL_ID_BTN;
		gui_element[GUI_EL_BTN_SETWP].id = EL_ID_BTN;
	gui_element[GUI_EL_MBTN_VIEW].id = EL_ID_MBTN;
	gui_element[GUI_EL_MBTN_SETTINGS].id = EL_ID_MBTN;
		gui_element[GUI_EL_BTN_CALTOUCH].id = EL_ID_BTN;
		gui_element[GUI_EL_BTN_RESET].id = EL_ID_BTN;
		gui_element[GUI_EL_SW_LIDAR].id = EL_ID_SW;
		gui_element[GUI_EL_SW_STRDEBUG].id = EL_ID_SW;
		gui_element[GUI_EL_SW_STRDEBUGOS].id = EL_ID_SW;
		gui_element[GUI_EL_SW_STRSLAMUI].id = EL_ID_SW;
		gui_element[GUI_EL_SW_STRERR].id = EL_ID_SW;

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

		gui_element[GUI_EL_SW_PROCESSEDVIEW].label = (char *)"Processed:";
		gui_element[GUI_EL_SW_PROCESSEDVIEW].action = &gui_el_event_sw_processedview;
		gui_element[GUI_EL_SW_PROCESSEDVIEW].x = PAGE_GRID_DIST;
		gui_element[GUI_EL_SW_PROCESSEDVIEW].y = gui_element[GUI_EL_SW_SHOWSCAN].y + gui_element[GUI_EL_SW_SHOWSCAN].heigth + PAGE_GRID_DIST;
		gui_element[GUI_EL_SW_PROCESSEDVIEW].state = SW_OFF;

		gui_element[GUI_EL_BTN_CLEARMAP].label = (char *)"Clear map";
		gui_element[GUI_EL_BTN_CLEARMAP].action = &gui_el_event_btn_clearMap;
		gui_element[GUI_EL_BTN_CLEARMAP].x = PAGE_GRID_DIST;
		gui_element[GUI_EL_BTN_CLEARMAP].y = gui_element[GUI_EL_SW_PROCESSEDVIEW].y + gui_element[GUI_EL_SW_PROCESSEDVIEW].heigth + PAGE_GRID_DIST;
		gui_element[GUI_EL_BTN_CLEARMAP].state = GUI_EL_INVISIBLE;

		gui_element[GUI_EL_BTN_SETWP].label = (char *)"Set waypoints";
		gui_element[GUI_EL_BTN_SETWP].action = &gui_el_event_btn_setWp;
		gui_element[GUI_EL_BTN_SETWP].x = PAGE_GRID_DIST;
		gui_element[GUI_EL_BTN_SETWP].y = gui_element[GUI_EL_BTN_CLEARMAP].y + gui_element[GUI_EL_BTN_CLEARMAP].heigth + PAGE_GRID_DIST;
		gui_element[GUI_EL_BTN_SETWP].state = GUI_EL_INVISIBLE;

		gui_element[GUI_EL_AREA_MAP].x = gui_element[GUI_EL_SW_STARTMAPPING].x + gui_element[GUI_EL_SW_STARTMAPPING].length + PAGE_GRID_DIST;
		gui_element[GUI_EL_AREA_MAP].y = gui_element[GUI_EL_SW_STARTMAPPING].y;
		gui_element[GUI_EL_AREA_MAP].length = GetMaxX() - gui_element[GUI_EL_AREA_MAP].x;
		gui_element[GUI_EL_AREA_MAP].heigth = GetMaxY() - gui_element[GUI_EL_AREA_MAP].y;
		gui_element[GUI_EL_AREA_MAP].action = &gui_el_event_area_map;
		gui_element[GUI_EL_AREA_MAP].state = MAP_ACTIVE;

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

		gui_element[GUI_EL_SW_STRDEBUG].x = PAGE_GRID_DIST;
		gui_element[GUI_EL_SW_STRDEBUG].y = gui_element[GUI_EL_SW_LIDAR].y + gui_element[GUI_EL_SW_LIDAR].heigth + 5;
		gui_element[GUI_EL_SW_STRDEBUG].label = (char *)"Debug:";
		gui_element[GUI_EL_SW_STRDEBUG].action = &gui_el_event_sw_strdebug;
		gui_element[GUI_EL_SW_STRDEBUG].state = GUI_EL_INVISIBLE;

		gui_element[GUI_EL_SW_STRDEBUGOS].x = PAGE_GRID_DIST;
		gui_element[GUI_EL_SW_STRDEBUGOS].y = gui_element[GUI_EL_SW_STRDEBUG].y + gui_element[GUI_EL_SW_STRDEBUG].heigth + 5;
		gui_element[GUI_EL_SW_STRDEBUGOS].label = (char *)"Debug OS:";
		gui_element[GUI_EL_SW_STRDEBUGOS].action = &gui_el_event_sw_strdebugos;
		gui_element[GUI_EL_SW_STRDEBUGOS].state = GUI_EL_INVISIBLE;

		gui_element[GUI_EL_SW_STRERR].x = PAGE_GRID_DIST;
		gui_element[GUI_EL_SW_STRERR].y = gui_element[GUI_EL_SW_STRDEBUGOS].y + gui_element[GUI_EL_SW_STRDEBUGOS].heigth + 5;
		gui_element[GUI_EL_SW_STRERR].label = (char *)"Stream err:";
		gui_element[GUI_EL_SW_STRERR].action = &gui_el_event_sw_strerr;
		gui_element[GUI_EL_SW_STRERR].state = GUI_EL_INVISIBLE;

		gui_element[GUI_EL_SW_STRSLAMUI].x = PAGE_GRID_DIST;
		gui_element[GUI_EL_SW_STRSLAMUI].y = gui_element[GUI_EL_SW_STRERR].y + gui_element[GUI_EL_SW_STRERR].heigth + 5;
		gui_element[GUI_EL_SW_STRSLAMUI].label = (char *)"SlamUI:";
		gui_element[GUI_EL_SW_STRSLAMUI].action = &gui_el_event_sw_strslamui;
		gui_element[GUI_EL_SW_STRSLAMUI].state = GUI_EL_INVISIBLE;


	for(u8 i = 0; i < STAT_STACK_SIZE; i++)
		statusbar_addMessage((char *) " ", LCD_COLOR_WHITE);

	statusbar_addMessage((char *) "Jugend Forscht 2015 - System started.", LCD_COLOR_WHITE);

	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO); //Calibrate Touchscreen with this Button
}

u8 mapping = 0; //Is the robot running and mapping or is it waiting for the start?
u8 setWaypoints = 0; //Is it currently allowed to set the waypoints in the map or can you set the robot position?
u8 processedView = 0;//Processed or raw view of the map?
int8_t batt_percent_old = 100; //Last battery percent state (refresh statusbar if changing)

portTASK_FUNCTION( vGUITask, pvParameters )
{
	portTickType xLastWakeTime;

	foutf(&debugOS, "xTask GUI started.\n");

	xLastWakeTime = xTaskGetTickCount();

	u8 timer_drawMap = 0;

	for(;;)
	{
		//foutf(&debugOS, "Watermark gui: %i\n", uxTaskGetStackHighWaterMark( NULL ));

		gui_handler(gui_element);

		if(STM_EVAL_PBGetState(BUTTON_USER))
		{
			menu = MENU_CALIBRATION;
		}

		if(battery.percent != batt_percent_old) //Redraw statusbar if battery value changes
		{
			gui_drawAREAstatusbar(&gui_element[GUI_EL_AREA_STATUSBAR_TOP]);
			batt_percent_old = battery.percent;
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
			//SW Show scan
			//BTN Clear map
			//BTN Set waypoint

			gui_element[GUI_EL_AREA_MAP].state = GUI_EL_INTOUCHABLE;
			if(mapping)			gui_element[GUI_EL_SW_STARTMAPPING].state = SW_ON;
			else				gui_element[GUI_EL_SW_STARTMAPPING].state = SW_OFF;
			if(show_scan)		gui_element[GUI_EL_SW_SHOWSCAN].state = SW_ON;
			else				gui_element[GUI_EL_SW_SHOWSCAN].state = SW_OFF;
			if(processedView)	gui_element[GUI_EL_SW_PROCESSEDVIEW].state = SW_ON;
			else				gui_element[GUI_EL_SW_PROCESSEDVIEW].state = SW_OFF;

			gui_element[GUI_EL_AREA_MAP].state = MAP_ACTIVE;

			gui_element[GUI_EL_BTN_CLEARMAP].state = BTN_NOT_ACTIVE;

			if(setWaypoints)	gui_element[GUI_EL_BTN_SETWP].state = BTN_ACTIVE;
			else				gui_element[GUI_EL_BTN_SETWP].state = BTN_NOT_ACTIVE;

			gui_drawSW(&gui_element[GUI_EL_SW_STARTMAPPING]);
			gui_drawSW(&gui_element[GUI_EL_SW_SHOWSCAN]);
			gui_drawSW(&gui_element[GUI_EL_SW_PROCESSEDVIEW]);
			gui_drawBTN(&gui_element[GUI_EL_BTN_CLEARMAP]);
			gui_drawBTN(&gui_element[GUI_EL_BTN_SETWP]);

			timer_drawMap = 0;

			menu = MENU_MAP_IDLE;

		case MENU_MAP_IDLE:

			if(timer_drawMap == 0)
			{
				gui_drawAREAmap(&gui_element[GUI_EL_AREA_MAP]);
				timer_drawMap = MAP_REFRESHTIME;
			}
			timer_drawMap --;

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

			if(debug.active)	gui_element[GUI_EL_SW_STRDEBUG].state = SW_ON; //Stream on/off switches
			else				gui_element[GUI_EL_SW_STRDEBUG].state = SW_OFF;
			gui_drawSW(&gui_element[GUI_EL_SW_STRDEBUG]);
			if(debugOS.active)	gui_element[GUI_EL_SW_STRDEBUGOS].state = SW_ON;
			else				gui_element[GUI_EL_SW_STRDEBUGOS].state = SW_OFF;
			gui_drawSW(&gui_element[GUI_EL_SW_STRDEBUGOS]);
			if(error.active)	gui_element[GUI_EL_SW_STRERR].state = SW_ON;
			else				gui_element[GUI_EL_SW_STRERR].state = SW_OFF;
			gui_drawSW(&gui_element[GUI_EL_SW_STRERR]);
			if(slamUI.active)	gui_element[GUI_EL_SW_STRSLAMUI].state = SW_ON;
			else				gui_element[GUI_EL_SW_STRSLAMUI].state = SW_OFF;
			gui_drawSW(&gui_element[GUI_EL_SW_STRSLAMUI]);

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

		vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_RATE_MS ) );
	}
}
