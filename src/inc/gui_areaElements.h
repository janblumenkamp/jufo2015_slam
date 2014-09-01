#ifndef GUI_AREAELEMENTS_H
#define GUI_AREAELEMENTS_H

#include "stm32f4xx.h"

enum GUI_ELEMENT_STAT_STATES {
	STAT_NOT_DROPPED = 2, STAT_HOVERED, STAT_DROPPED
};

enum GUI_ELEMENT_SCAN_STATES {
	SCAN_BUSY = 2, SCAN_OK
};

#define STAT_STACK_SIZE 25

typedef struct {
	char *message[STAT_STACK_SIZE];
	u16 color_bg[STAT_STACK_SIZE];
	u8 index;
} GUI_ELEMENT_STAT_STACK;

extern void statusbar_addMessage(char *message, u16 color_bg);

extern void gui_drawAREAstatusbar(GUI_ELEMENT *element);

extern void gui_drawAREAscan(GUI_ELEMENT *element);

#endif // GUI_AREAELEMENTS_H
