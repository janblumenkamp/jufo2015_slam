////////////////////////////////////////////////////////////////////////////////
/// gui_graphics.h - Elementar definitions of Data structures etc. of the
/// GUI Elements
///
/// ADDING NEW ELEMENTS: see gui.h
/// LIST OF ALL AVAILABLE ELEMENTS:
///
/// Menubutton:
///        Label     /
///   –––––––––––––––
///
/// Button:
///  –––––––––––––––
/// |   Label       |:
///  ––––––––––––––– :
/// ..................
///
/// Switch:
/// ––––––––––––––––
/// | |on|    off  |
/// ––––––––––––––––
///
/// Slider:
///
/// ––––––––––––––––––––––––––––––––
/// |       |25%|                  |
/// –––––––––––––––––––––––––––––––
///
/// Area: Userdefined, see gui_areaElements.c
//////////////////////////////////////////////////////////////////////////////////////

#ifndef GUI_GRAPHICS_H
#define GUI_GRAPHICS_H

#include "stm32f4xx.h"
#include "gui.h"
#include "SSD1963_api.h"
#include "stm32_ub_font.h"

//Color Definitions
#define GUI_COLOR_BACKGROUND	LCD_COLOR_WHITE
#define GUI_COLOR_FONT			LCD_COLOR_BLACK
#define GUI_COLOR_EL_ACTIVE		LCD_COLOR_GRAY4
#define GUI_COLOR_EL_UNACTIVE	LCD_COLOR_GRAY2

////////////////////////////////////
/// \brief The GUI_ELEMENT_ID enum
/// Possible IDs of GUI_ELEMENT.id (is the gui element a button, a switch...)

enum GUI_ELEMENT_ID {
	EL_ID_AREA, //Area, free space with touchscreen support for own graphics
	EL_ID_MBTN, //MenuBuTtoN
	EL_ID_BTN, //BuTtoN
	EL_ID_SW, //SWitch
	EL_ID_SLI //SLIder
};

/// Disables the event functions of the not-active elements
///	(if the page is changed or the statusbar is dropped, all elements are
///	are firstly set as inactive and then in the menu statemachine corresponding
///	init state of the new page resetted to the active state), also elements are not drawn
/// if element state is set to invisible

#define GUI_EL_INVISIBLE 0
#define GUI_EL_INTOUCHABLE 1 //Visible, but not touchable

//if not invisible, the element is one of the following states:
enum GUI_ELEMENT_AREA_STATES {
	//You can also define your own states for your area.
	AREA_NOT_ACTIVE = 2, AREA_HOVERED, AREA_ACTIVE
};

enum GUI_ELEMENT_MBTN_STATES {
	MBTN_NOT_ACTIVE = 2, MBTN_HOVERED, MBTN_ACTIVE
};

enum GUI_ELEMENT_BTN_STATES {
	BTN_NOT_ACTIVE = 2, BTN_HOVERED, BTN_OK, BTN_BUSY, BTN_ACTIVE
};

enum GUI_ELEMENT_SW_STATES {
	SW_OFF = 2, SW_BUSY, SW_ON
};

/*enum GUI_ELEMENT_SLI_STATES {
	//0 to 100%
};*/

//This event can occur and are transmitted to the event-functions:
typedef struct {
	u8 pressed:1;
	u8 clicked:1;
	u8 released:1;
	u8 doubleclick:1;
} ELEMENT_EVENT;

//Struct of the gui element:
typedef struct {
	int16_t x;
	int16_t y;
	int16_t length;
	int16_t heigth;
	char *label;
	UB_pFont *font;
	ELEMENT_EVENT event;
	void (*action)(ELEMENT_EVENT *event); //Called if pressed by touchscreen
	u8 id;
	u8 state;
} GUI_ELEMENT;

//Standard Initialisation of the GUI_ELEMENTS
extern void graphics_init(GUI_ELEMENT element[]);

//handles (touch) events
extern void gui_handler(GUI_ELEMENT areas[]);

//clears (sets to background color) the size of the given element
extern void gui_clearAREA(GUI_ELEMENT *element);

//Draw Menubutton; Standard settings (if not set to another value in gui_init
#define MBTN_STD_LENGTH	100
#define MBTN_STD_HEIGHT	30
extern void gui_drawMBTN(GUI_ELEMENT *element);

//Draw Button, Standard Settings...
#define BTN_STD_LENGTH 160
#define BTN_STD_HEIGHT 30
extern void gui_drawBTN(GUI_ELEMENT *element);

//Draw Switch, Standard Settings...
#define SW_STD_LENGTH 160
#define SW_STD_HEIGHT 30
extern void gui_drawSW(GUI_ELEMENT *element);

//TO DO: Draw Slider, Standard Settings...
#define SLI_STD_LENGTH 200
#define SLI_STD_HEIGHT 30
extern void gui_drawSLI(GUI_ELEMENT *element);

#endif // GUI_GRAPHICS_H
