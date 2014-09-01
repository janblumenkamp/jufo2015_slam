#ifndef GUI_GRAPHICS_H
#define GUI_GRAPHICS_H

#include "stm32f4xx.h"
#include "gui.h"
#include "SSD1963_api.h"
/* Menubutton:
 *        Label     /
 *   –––––––––––––––
 *
 * Button:
 *  –––––––––––––––
 * |   Label       |:
 *  ––––––––––––––– :
 * ..................
 *
 * Switch:
 * ––––––––––––––––
 * | |on|    off  |
 * ––––––––––––––––
 *
 * Slider:
 *
 * ––––––––––––––––––––––––––––––––
 * |       |25%|                  |
 * –––––––––––––––––––––––––––––––
 */

#define GUI_COLOR_BACKGROUND	LCD_COLOR_WHITE
#define GUI_COLOR_FONT			LCD_COLOR_BLACK
#define GUI_COLOR_EL_ACTIVE		LCD_COLOR_CYAN
#define GUI_COLOR_EL_UNACTIVE	LCD_COLOR_GRAY2

enum GUI_ELEMENT_ID {
	EL_ID_AREA, //Area, free space with touchscreen support for own graphics
	EL_ID_MBTN, //MenuBuTtoN
	EL_ID_BTN, //BuTtoN
	EL_ID_SW, //SWitch
	EL_ID_SLI //SLIder
};

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

typedef struct {
	u8 pressed:1;
	u8 clicked:1;
	u8 released:1;
	u8 doubleclick:1;
	u8 redraw:1;
} ELEMENT_EVENT;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t length;
	int16_t heigth;
	char *label;
	ELEMENT_EVENT event;
	void (*action)(ELEMENT_EVENT *event); //Called if pressed by touchscreen
	u8 id;
	u8 state;
} GUI_ELEMENT;

extern void init_graphics(GUI_ELEMENT element[]);

extern void gui_handler(GUI_ELEMENT areas[]);

extern void gui_clearAREA(GUI_ELEMENT *element);

#define MBTN_STD_LENGTH	100
#define MBTN_STD_HEIGHT	30
extern void gui_drawMBTN(GUI_ELEMENT *element);

#define BTN_STD_LENGTH 160
#define BTN_STD_HEIGHT 30
extern void gui_drawBTN(GUI_ELEMENT *element);

#define SW_STD_LENGTH 160
#define SW_STD_HEIGHT 30
extern void gui_drawSW(GUI_ELEMENT *element);

#define SLI_STD_LENGTH 200
#define SLI_STD_HEIGHT 30
extern void gui_drawSLI(GUI_ELEMENT *element);

#endif // GUI_GRAPHICS_H
