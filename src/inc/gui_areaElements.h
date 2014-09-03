////////////////////////////////////////////////////////////////////////////////
/// gui_areaElements.h - Elementar definitions of Data structures etc. of the
/// GUI Element area!
///
/// ADDING NEW ELEMENTS: see gui.h
//////////////////////////////////////////////////////////////////////////////////////

#ifndef GUI_AREAELEMENTS_H
#define GUI_AREAELEMENTS_H

#include "stm32f4xx.h"

extern u8 show_scan; //Display lidar scan or don’t?

////////////////////////////////////////////
/// \brief The GUI_ELEMENT_STAT_STATES enum
/// Definition of states of area statusbar

enum GUI_ELEMENT_STAT_STATES {
	STAT_NOT_DROPPED = 2, STAT_HOVERED, STAT_DROPPED
};

/// Definition of the states of the Area of the map
/*enum GUI_ELEMENT_MAP_STATES {

};*/

//Stacksize of the Statusbar (amount of Status messages in the statusbar history)
#define STAT_STACK_SIZE 25

//Stack struct of the statusbar history
typedef struct {
	char *message[STAT_STACK_SIZE];
	u16 color_bg[STAT_STACK_SIZE];
	u8 index;
} GUI_ELEMENT_STAT_STACK;

//Adds new message to the Stack of the Statusbar
extern void statusbar_addMessage(char *message, u16 color_bg);

//Draws Statusbar
extern void gui_drawAREAstatusbar(GUI_ELEMENT *element);

//Draws Map
extern void gui_drawAREAmap(GUI_ELEMENT *element);

#endif // GUI_AREAELEMENTS_H
